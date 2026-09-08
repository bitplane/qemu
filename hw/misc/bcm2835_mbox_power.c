// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM2835 mailbox channel 0 (VideoCore power management)
 *
 * Channel 0 is the legacy interface an ARM-side client uses to ask the
 * VideoCore to power a peripheral up or down. It predates, and is
 * independent of, the property-channel SET_POWER_STATE tag.
 *
 * Wire format, per the reference client in the Raspberry Pi Linux tree
 * (arch/arm/mach-bcm2708/power.c, bcm_power_request()):
 *
 *     bcm_mailbox_write(MBOX_CHAN_POWER, global_request << 4);
 *     rc = bcm_mailbox_read(MBOX_CHAN_POWER, &actual);
 *     actual >>= 4;
 *
 * i.e. the 28-bit payload is a device bitmask shifted left by four (the
 * low nibble carries the channel number, as on every mailbox channel),
 * and the VideoCore answers on the same channel with the power state it
 * actually reached. The client treats "actual != requested" as a failure.
 *
 * The bit assignments come from arch/arm/mach-bcm2708/include/mach/
 * arm_power.h in the same tree: SDCARD 0, UART 1, MINIUART 2, USB 3,
 * I2C0 4, I2C1 5, I2C2 6, SPI 7, CCP2TX 8.
 *
 * Nothing here is actually powered or unpowered, so every request is
 * granted and the reply always mirrors the request. That is enough for
 * guests that block waiting for the acknowledgement -- RISC OS asks for
 * BCM_POWER_USB (bit 3, i.e. a payload of 0x80) from its HAL before it
 * will continue booting, and without a reply it spins forever.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/misc/bcm2835_mbox_power.h"
#include "hw/misc/bcm2835_mbox_defs.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

/* BCM_POWER_MASK: (1 << BCM_POWER_MAX) - 1, with BCM_POWER_MAX == 9 */
#define BCM2835_POWER_MASK 0x1ff

/* The payload is the device bitmask shifted up past the channel nibble. */
#define BCM2835_POWER_SHIFT 4

static uint64_t bcm2835_mbox_power_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    BCM2835MboxPowerState *s = opaque;
    uint32_t res = 0;

    switch (offset) {
    case MBOX_AS_DATA:
        res = s->reply;
        s->pending = false;
        qemu_set_irq(s->mbox_irq, 0);
        break;

    case MBOX_AS_PENDING:
        res = s->pending;
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset %"HWADDR_PRIx"\n",
                      __func__, offset);
        return 0;
    }

    return res;
}

static void bcm2835_mbox_power_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    BCM2835MboxPowerState *s = opaque;
    uint32_t request;

    switch (offset) {
    case MBOX_AS_DATA:
        /* bcm2835_mbox should check our pending status before pushing */
        assert(!s->pending);

        request = (uint32_t)value >> BCM2835_POWER_SHIFT;
        if (request & ~BCM2835_POWER_MASK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: unknown power domain(s) requested 0x%x\n",
                          __func__, request & ~BCM2835_POWER_MASK);
            request &= BCM2835_POWER_MASK;
        }

        /*
         * Everything the guest can ask for is always available, so the
         * state reached is the state requested.
         */
        s->state = request;
        s->reply = (s->state << BCM2835_POWER_SHIFT) | MBOX_CHAN_POWER;
        s->pending = true;
        qemu_set_irq(s->mbox_irq, 1);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset %"HWADDR_PRIx"\n",
                      __func__, offset);
        return;
    }
}

static const MemoryRegionOps bcm2835_mbox_power_ops = {
    .read = bcm2835_mbox_power_read,
    .write = bcm2835_mbox_power_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const VMStateDescription vmstate_bcm2835_mbox_power = {
    .name = TYPE_BCM2835_MBOX_POWER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(state, BCM2835MboxPowerState),
        VMSTATE_UINT32(reply, BCM2835MboxPowerState),
        VMSTATE_BOOL(pending, BCM2835MboxPowerState),
        VMSTATE_END_OF_LIST()
    }
};

static void bcm2835_mbox_power_init(Object *obj)
{
    BCM2835MboxPowerState *s = BCM2835_MBOX_POWER(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &bcm2835_mbox_power_ops, s,
                          TYPE_BCM2835_MBOX_POWER, 0x10);

    /*
     * bcm2835_mbox_power_ops call into bcm2835_mbox, which in-turn reads
     * from iomem. As such, mark iomem as re-entrancy safe (the property
     * channel does the same for the same reason).
     */
    s->iomem.disable_reentrancy_guard = true;

    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->mbox_irq);
}

static void bcm2835_mbox_power_reset(DeviceState *dev)
{
    BCM2835MboxPowerState *s = BCM2835_MBOX_POWER(dev);

    s->state = 0;
    s->reply = 0;
    s->pending = false;
}

static void bcm2835_mbox_power_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, bcm2835_mbox_power_reset);
    dc->vmsd = &vmstate_bcm2835_mbox_power;
}

static const TypeInfo bcm2835_mbox_power_info = {
    .name          = TYPE_BCM2835_MBOX_POWER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2835MboxPowerState),
    .class_init    = bcm2835_mbox_power_class_init,
    .instance_init = bcm2835_mbox_power_init,
};

static void bcm2835_mbox_power_register_types(void)
{
    type_register_static(&bcm2835_mbox_power_info);
}

type_init(bcm2835_mbox_power_register_types)
