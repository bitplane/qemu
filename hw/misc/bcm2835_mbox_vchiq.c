// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM2835 VCHIQ peer (mailbox channel 3 + the VCHIQ doorbells)
 *
 * VCHIQ is the shared-memory message channel between the ARM and the
 * VideoCore. The ARM hands over a page-aligned "slot zero" area by writing
 * its bus address to mailbox channel 3; from then on the two sides exchange
 * messages through that area and poke each other with doorbells, not with
 * the mailbox. Guests that expect a VideoCore therefore cannot be satisfied
 * by the mailbox alone: the channel-3 write is fire-and-forget, and the
 * reply word is never read.
 *
 * That matters because a guest which asks VCHIQ to connect and gets no
 * answer does not fail -- it waits forever. RISC OS is the case in point:
 * BCMVideo's initialisation calls VCHIQ_Connect before it registers its
 * GraphicsV driver, and VCHIQ_Connect ends in an unbounded semaphore wait
 * with no timeout, so the boot stops dead and no display is ever set up.
 *
 * This device is the smallest thing that unblocks such a guest honestly. It
 * completes the CONNECT handshake and then refuses every service the guest
 * tries to open, by answering each OPEN with a CLOSE. That is a state the
 * protocol defines and guests are written to cope with -- it presents a
 * VideoCore that is alive but offers no services, rather than one that is
 * dead. Emulating the services themselves (vcgencmd, dispmanx, TV service,
 * audio) is deliberately out of scope; a guest whose OPEN succeeds would
 * immediately expect all of them.
 *
 * Layout and protocol constants below are taken from the VCHIQ interface
 * headers as shipped in the RISC OS BCM2835 source tree
 * (RiscOS/Sources/HWSupport/VCHIQ/vc04_services/interface/vchiq_arm/, the
 * same Broadcom code the Linux driver is built from):
 *
 *   h/vchiq_if:41       VCHIQ_SLOT_SIZE 4096
 *   h/vchiq_if:45-46    VCHIQ_MAKE_FOURCC(x0,x1,x2,x3) =
 *                         (x0 << 24) | (x1 << 16) | (x2 << 8) | x3
 *   h/vchiq_cfg:37      VCHIQ_MAGIC = VCHIQ_MAKE_FOURCC('V','C','H','I')
 *   h/vchiq_core:100-103  VCHIQ_MSG_CONNECT 1, VCHIQ_MSG_OPEN 2,
 *                         VCHIQ_MSG_OPENACK 3, VCHIQ_MSG_CLOSE 4
 *   h/vchiq_core:118-123  VCHIQ_MAKE_MSG(type,srcport,dstport) =
 *                         (type << 24) | (srcport << 12) | dstport
 *   h/vchiq_core:268-273  REMOTE_EVENT_T { int armed; int fired; u32 event; }
 *   h/vchiq_core:350-403  VCHIQ_SHARED_STATE_T and VCHIQ_SLOT_ZERO_T
 *   h/vchiq_if:81-91      VCHIQ_HEADER_T { int msgid; unsigned int size; }
 *
 * The doorbell register offsets come from the guest's own HAL header,
 * RiscOS/Sources/HAL/HAL_BCM2835/hdr/BCM2835:188-191 -- MB_Base &b800,
 * MB_DBell0 &40 (VC -> ARM), MB_DBell2 &48 (ARM -> VC).
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/misc/bcm2835_mbox_vchiq.h"
#include "hw/misc/bcm2835_mbox_defs.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/dma.h"
#include "trace.h"

#define VCHIQ_MAGIC             0x56434849  /* 'V','C','H','I' */
#define VCHIQ_SLOT_SIZE         4096

#define VCHIQ_MSG_CONNECT       1
#define VCHIQ_MSG_OPEN          2
#define VCHIQ_MSG_CLOSE         4

#define VCHIQ_MSG_TYPE(msgid)   ((uint32_t)(msgid) >> 24)
#define VCHIQ_MSG_SRCPORT(msgid) (((uint32_t)(msgid) >> 12) & 0xfff)
#define VCHIQ_MAKE_MSG(type, srcport, dstport) \
    (((uint32_t)(type) << 24) | ((uint32_t)(srcport) << 12) | \
     (uint32_t)(dstport))

/* VCHIQ_SLOT_ZERO_T field offsets */
#define SZ_MAGIC                0
#define SZ_VERSION              4   /* short */
#define SZ_VERSION_MIN          6   /* short */
#define SZ_SLOT_ZERO_SIZE       8
#define SZ_SLOT_SIZE            12
#define SZ_MAX_SLOTS            16
#define SZ_MAX_SLOTS_PER_SIDE   20
#define SZ_PLATFORM_DATA        24  /* int[2] */
#define SZ_HEADER_SIZE          32

/* VCHIQ_SHARED_STATE_T field offsets */
#define SS_INITIALISED          0
#define SS_SLOT_FIRST           4
#define SS_SLOT_LAST            8
#define SS_SLOT_SYNC            12
#define SS_TRIGGER_ARMED        16
#define SS_TRIGGER_FIRED        20
#define SS_TRIGGER_EVENT        24
#define SS_TX_POS               28
#define SS_RECYCLE_ARMED        32
#define SS_RECYCLE_FIRED        36
#define SS_RECYCLE_EVENT        40
#define SS_SLOT_QUEUE_RECYCLE   44
#define SS_SYNC_TRIGGER         48  /* REMOTE_EVENT_T */
#define SS_SYNC_RELEASE         60  /* REMOTE_EVENT_T */
#define SS_SLOT_QUEUE           72  /* int[max_slots_per_side] */

/*
 * VCHIQ_SLOT_INFO_T is two shorts, so the trailing slots[] array is 4 bytes
 * per entry (h/vchiq_core:283-287).
 */
#define SLOT_INFO_SIZE          4

/* Doorbell window (relative to its own MemoryRegion, based at MB_Base+0x40) */
#define DBELL_VC_TO_ARM         0x00    /* MB_DBell0 */
#define DBELL_1                 0x04    /* MB_DBell1 */
#define DBELL_ARM_TO_VC         0x08    /* MB_DBell2 */
#define DBELL_3                 0x0c    /* MB_DBell3 */

/*
 * The bit the guest tests after reading doorbell 0 to decide that there are
 * messages to collect (VCHIQ/c/cmodule: "if (status & 0x4)").
 */
#define DBELL_HAVE_MESSAGES     0x4

static uint32_t vchiq_ldl(BCM2835MboxVchiqState *s, uint32_t addr)
{
    return address_space_ldl_le(&s->dma_as, addr, MEMTXATTRS_UNSPECIFIED,
                                NULL);
}

static void vchiq_stl(BCM2835MboxVchiqState *s, uint32_t addr, uint32_t val)
{
    address_space_stl_le(&s->dma_as, addr, val, MEMTXATTRS_UNSPECIFIED, NULL);
}

/*
 * Message stride: sizeof(VCHIQ_HEADER_T) is 8 here (the flexible data[]
 * member is compiled out for this guest), and messages are 8-byte aligned
 * (calc_stride() in c/vchiq_core).
 */
static uint32_t vchiq_stride(uint32_t size)
{
    return (size + 8 + 7) & ~7u;
}

static void vchiq_ring_arm_doorbell(BCM2835MboxVchiqState *s)
{
    /* The slot handler only runs if its trigger event is signalled. */
    vchiq_stl(s, s->slave + SS_TRIGGER_FIRED, 1);
    s->doorbell0 |= DBELL_HAVE_MESSAGES;
    qemu_set_irq(s->irq, 1);
}

/*
 * Append a message to the master's stream. Returns false (and sends
 * nothing) if it would cross a slot boundary -- this device only ever
 * queues a CONNECT and a handful of CLOSEs, far less than one slot, so
 * needing to recycle a slot means something is wrong and it is better to
 * stop than to half-implement the recycle protocol.
 */
static bool vchiq_queue_message(BCM2835MboxVchiqState *s, uint32_t msgid,
                                uint32_t size)
{
    uint32_t stride = vchiq_stride(size);
    uint32_t stream_slot = s->tx_pos / s->slot_size;
    uint32_t offset = s->tx_pos % s->slot_size;
    uint32_t index, slot, header;

    if (offset + stride > s->slot_size) {
        qemu_log_mask(LOG_UNIMP, "%s: master slot full, refusing to wrap "
                      "(tx_pos 0x%x)\n", __func__, s->tx_pos);
        return false;
    }
    if (s->slot_first + stream_slot > s->slot_last) {
        qemu_log_mask(LOG_UNIMP, "%s: out of master slots\n", __func__);
        return false;
    }

    /* slot_queue is a circular buffer, indexed the same way on both sides. */
    index = stream_slot & (s->max_slots_per_side - 1);
    slot = vchiq_ldl(s, s->master + SS_SLOT_QUEUE + 4 * index);
    if (slot < s->slot_first || slot > s->slot_last) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid master slot %u\n",
                      __func__, slot);
        return false;
    }
    header = s->slot_zero + slot * s->slot_size + offset;

    vchiq_stl(s, header, msgid);
    vchiq_stl(s, header + 4, size);

    s->tx_pos += stride;
    vchiq_stl(s, s->master + SS_TX_POS, s->tx_pos);

    trace_bcm2835_vchiq_tx(VCHIQ_MSG_TYPE(msgid), msgid, size, header);
    return true;
}

/*
 * Take the slot zero area the guest just handed over, check it is the shape
 * we know, fill in the half the VideoCore is responsible for, and deliver a
 * CONNECT.
 */
static void vchiq_handover(BCM2835MboxVchiqState *s, uint32_t base)
{
    uint32_t magic, zero_size, slot_size, max_slots, per_side;
    uint32_t shared_size, i;

    if (s->active) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: second handover ignored (0x%08x)\n", __func__,
                      base);
        return;
    }

    magic = vchiq_ldl(s, base + SZ_MAGIC);
    if (magic != VCHIQ_MAGIC) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad VCHIQ magic 0x%08x at 0x%08x\n", __func__,
                      magic, base);
        return;
    }

    zero_size = vchiq_ldl(s, base + SZ_SLOT_ZERO_SIZE);
    slot_size = vchiq_ldl(s, base + SZ_SLOT_SIZE);
    max_slots = vchiq_ldl(s, base + SZ_MAX_SLOTS);
    per_side = vchiq_ldl(s, base + SZ_MAX_SLOTS_PER_SIDE);

    /*
     * Derive the size of each shared state rather than hardcoding it: the
     * struct ends in a debug[] array whose length is a build option, so the
     * only portable handle on the layout is the slot_zero_size the guest
     * itself computed.
     */
    if (slot_size != VCHIQ_SLOT_SIZE || per_side == 0 ||
        (per_side & (per_side - 1)) ||
        max_slots > (UINT32_MAX - base) / VCHIQ_SLOT_SIZE ||
        zero_size > VCHIQ_SLOT_SIZE ||
        per_side > max_slots ||
        zero_size < SZ_HEADER_SIZE + SLOT_INFO_SIZE * max_slots) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: implausible slot zero (size %u, slot %u, "
                      "slots %u/%u)\n", __func__, zero_size, slot_size,
                      per_side, max_slots);
        return;
    }
    shared_size = zero_size - SZ_HEADER_SIZE - SLOT_INFO_SIZE * max_slots;
    if (shared_size & 1) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: odd shared state size %u\n",
                      __func__, shared_size);
        return;
    }
    shared_size /= 2;
    if (shared_size < SS_SLOT_QUEUE + 4 * per_side) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: shared state too small (%u)\n", __func__,
                      shared_size);
        return;
    }

    s->slot_zero = base;
    s->slot_size = slot_size;
    s->max_slots_per_side = per_side;
    s->master = base + SZ_HEADER_SIZE;
    s->slave = s->master + shared_size;

    s->slot_first = vchiq_ldl(s, s->master + SS_SLOT_FIRST);
    s->slot_last = vchiq_ldl(s, s->master + SS_SLOT_LAST);
    s->slave_slot_first = vchiq_ldl(s, s->slave + SS_SLOT_FIRST);
    s->slave_slot_last = vchiq_ldl(s, s->slave + SS_SLOT_LAST);
    if (!s->slot_first || s->slot_last >= max_slots ||
        s->slot_last < s->slot_first ||
        !s->slave_slot_first || s->slave_slot_last >= max_slots ||
        s->slave_slot_last < s->slave_slot_first ||
        s->slave_slot_last - s->slave_slot_first >= per_side ||
        !(s->slot_last < s->slave_slot_first ||
          s->slave_slot_last < s->slot_first) ||
        s->slot_last - s->slot_first >= per_side) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad master slot range %u..%u\n", __func__,
                      s->slot_first, s->slot_last);
        return;
    }

    trace_bcm2835_vchiq_handover(base, zero_size, shared_size, s->slot_first,
                                 s->slot_last);

    /*
     * The guest zeroes the whole of slot zero and then fills in only its
     * own (slave) slot queue, leaving the master's for the VideoCore. If
     * the master queue stays zeroed the guest's parser would follow entry
     * zero straight back into slot zero itself.
     */
    for (i = 0; s->slot_first + i <= s->slot_last; i++) {
        vchiq_stl(s, s->master + SS_SLOT_QUEUE + 4 * i, s->slot_first + i);
    }

    /*
     * Keep the master trigger armed permanently. The guest only rings the
     * doorbell when the remote event it is signalling is armed, so leaving
     * this clear makes it queue messages silently and wait forever.
     */
    vchiq_stl(s, s->master + SS_TRIGGER_ARMED, 1);
    vchiq_stl(s, s->master + SS_RECYCLE_ARMED, 1);
    vchiq_stl(s, s->master + SS_INITIALISED, 1);

    s->tx_pos = 0;
    s->rx_pos = 0;
    s->active = true;

    /*
     * Deliver CONNECT straight away. The guest's connect semaphore counts,
     * so arriving before its own CONNECT is queued is harmless.
     */
    if (vchiq_queue_message(s, VCHIQ_MAKE_MSG(VCHIQ_MSG_CONNECT, 0, 0), 0)) {
        vchiq_ring_arm_doorbell(s);
    }
}

/* Consume everything the guest has queued since we last looked. */
static void vchiq_drain(BCM2835MboxVchiqState *s)
{
    uint32_t slave_tx;
    bool replied = false;
    uint32_t remaining;

    if (!s->active) {
        return;
    }

    /* The guest sets this when it signals us; acknowledge it. */
    vchiq_stl(s, s->master + SS_TRIGGER_FIRED, 0);

    slave_tx = vchiq_ldl(s, s->slave + SS_TX_POS);
    remaining = slave_tx - s->rx_pos;
    if (remaining > s->max_slots_per_side * s->slot_size) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid receive position\n",
                      __func__);
        return;
    }

    while (remaining) {
        uint32_t index, slot, header, msgid, size;
        uint32_t offset = s->rx_pos % s->slot_size;
        uint32_t stride;

        index = (s->rx_pos / s->slot_size) & (s->max_slots_per_side - 1);
        slot = vchiq_ldl(s, s->slave + SS_SLOT_QUEUE + 4 * index);
        if (slot < s->slave_slot_first || slot > s->slave_slot_last ||
            remaining < 8 || offset > s->slot_size - 8) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid receive slot\n",
                          __func__);
            break;
        }
        header = s->slot_zero + slot * s->slot_size +
                 (s->rx_pos % s->slot_size);

        msgid = vchiq_ldl(s, header);
        size = vchiq_ldl(s, header + 4);
        if (size > s->slot_size - offset - 8) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid message size\n",
                          __func__);
            break;
        }
        stride = vchiq_stride(size);
        if (stride > remaining || stride > s->slot_size - offset) {
            break;
        }

        trace_bcm2835_vchiq_rx(VCHIQ_MSG_TYPE(msgid), msgid, size, header);

        if (VCHIQ_MSG_TYPE(msgid) == VCHIQ_MSG_OPEN) {
            /*
             * Refuse the service. The guest treats a CLOSE received while
             * opening as "no such service" and gives up cleanly, which is
             * what every VCHIQ-dependent initialiser is written to handle.
             */
            uint32_t port = VCHIQ_MSG_SRCPORT(msgid);
            if (!vchiq_queue_message(s,
                    VCHIQ_MAKE_MSG(VCHIQ_MSG_CLOSE, 0, port), 0)) {
                break;
            }
            replied = true;
        }

        s->rx_pos += stride;
        remaining -= stride;
    }

    if (replied) {
        vchiq_ring_arm_doorbell(s);
    }
}

static uint64_t bcm2835_mbox_vchiq_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    switch (offset) {
    case MBOX_AS_DATA:
        /*
         * Nothing is ever queued back through the mailbox: channel 3 is
         * fire-and-forget and the guest never reads a reply.
         */
        return 0;

    case MBOX_AS_PENDING:
        /* Always zero, so bcm2835_mbox never tries to drain this child. */
        return 0;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset %"HWADDR_PRIx"\n",
                      __func__, offset);
        return 0;
    }
}

static void bcm2835_mbox_vchiq_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    BCM2835MboxVchiqState *s = opaque;

    switch (offset) {
    case MBOX_AS_DATA:
        vchiq_handover(s, (uint32_t)value & ~0xfu);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset %"HWADDR_PRIx"\n",
                      __func__, offset);
        break;
    }
}

static const MemoryRegionOps bcm2835_mbox_vchiq_ops = {
    .read = bcm2835_mbox_vchiq_read,
    .write = bcm2835_mbox_vchiq_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t bcm2835_vchiq_dbell_read(void *opaque, hwaddr offset,
                                         unsigned size)
{
    BCM2835MboxVchiqState *s = opaque;
    uint32_t res;

    switch (offset) {
    case DBELL_VC_TO_ARM:
        /* Read to clear -- the guest reads this from its IRQ handler. */
        res = s->doorbell0;
        s->doorbell0 = 0;
        qemu_set_irq(s->irq, 0);
        trace_bcm2835_vchiq_doorbell_read(res);
        return res;

    case DBELL_1:
    case DBELL_ARM_TO_VC:
    case DBELL_3:
        return 0;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset %"HWADDR_PRIx"\n",
                      __func__, offset);
        return 0;
    }
}

static void bcm2835_vchiq_dbell_write(void *opaque, hwaddr offset,
                                      uint64_t value, unsigned size)
{
    BCM2835MboxVchiqState *s = opaque;

    switch (offset) {
    case DBELL_ARM_TO_VC:
        trace_bcm2835_vchiq_doorbell_write((uint32_t)value);
        vchiq_drain(s);
        break;

    case DBELL_VC_TO_ARM:
    case DBELL_1:
    case DBELL_3:
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset %"HWADDR_PRIx"\n",
                      __func__, offset);
        break;
    }
}

static const MemoryRegionOps bcm2835_vchiq_dbell_ops = {
    .read = bcm2835_vchiq_dbell_read,
    .write = bcm2835_vchiq_dbell_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const VMStateDescription vmstate_bcm2835_mbox_vchiq = {
    .name = TYPE_BCM2835_MBOX_VCHIQ,
    .unmigratable = true,
};

static void bcm2835_mbox_vchiq_init(Object *obj)
{
    BCM2835MboxVchiqState *s = BCM2835_MBOX_VCHIQ(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &bcm2835_mbox_vchiq_ops, s,
                          TYPE_BCM2835_MBOX_VCHIQ, 0x10);
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);

    memory_region_init_io(&s->dbell, OBJECT(s), &bcm2835_vchiq_dbell_ops, s,
                          TYPE_BCM2835_MBOX_VCHIQ "-doorbell", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->dbell);

    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void bcm2835_mbox_vchiq_reset(DeviceState *dev)
{
    BCM2835MboxVchiqState *s = BCM2835_MBOX_VCHIQ(dev);

    s->active = false;
    s->slot_zero = 0;
    s->master = 0;
    s->slave = 0;
    s->slot_size = VCHIQ_SLOT_SIZE;
    s->max_slots_per_side = 0;
    s->slot_first = 0;
    s->slot_last = 0;
    s->slave_slot_first = 0;
    s->slave_slot_last = 0;
    s->tx_pos = 0;
    s->rx_pos = 0;
    s->doorbell0 = 0;
    qemu_set_irq(s->irq, 0);
}

static void bcm2835_mbox_vchiq_realize(DeviceState *dev, Error **errp)
{
    BCM2835MboxVchiqState *s = BCM2835_MBOX_VCHIQ(dev);
    Object *obj;

    obj = object_property_get_link(OBJECT(dev), "dma-mr", &error_abort);
    s->dma_mr = MEMORY_REGION(obj);
    address_space_init(&s->dma_as, s->dma_mr,
                       TYPE_BCM2835_MBOX_VCHIQ "-memory");

    bcm2835_mbox_vchiq_reset(dev);
}

static void bcm2835_mbox_vchiq_unrealize(DeviceState *dev)
{
    BCM2835MboxVchiqState *s = BCM2835_MBOX_VCHIQ(dev);

    address_space_destroy(&s->dma_as);
}

static void bcm2835_mbox_vchiq_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bcm2835_mbox_vchiq_realize;
    dc->unrealize = bcm2835_mbox_vchiq_unrealize;
    device_class_set_legacy_reset(dc, bcm2835_mbox_vchiq_reset);
    dc->vmsd = &vmstate_bcm2835_mbox_vchiq;
}

static const TypeInfo bcm2835_mbox_vchiq_info = {
    .name          = TYPE_BCM2835_MBOX_VCHIQ,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2835MboxVchiqState),
    .class_init    = bcm2835_mbox_vchiq_class_init,
    .instance_init = bcm2835_mbox_vchiq_init,
};

static void bcm2835_mbox_vchiq_register_types(void)
{
    type_register_static(&bcm2835_mbox_vchiq_info);
}

type_init(bcm2835_mbox_vchiq_register_types)
