// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SMSC LAN9500/LAN9512 USB 2.0 Ethernet controller
 *
 * This is the NIC fitted to a Raspberry Pi 1/2/3 -- it hangs off the SoC's
 * internal USB hub rather than sitting on a bus of its own -- so it is the
 * adapter that operating systems targeting those boards actually carry a
 * driver for. QEMU's raspi machines have never modelled it, which leaves
 * such a guest with no network at all unless it happens to also support
 * CDC/RNDIS (Linux does; RISC OS, for one, does not).
 *
 * Only what a driver touches is modelled: the vendor register window, the
 * EEPROM used to publish the MAC address, an internal PHY at MDIO address
 * 1, and the bulk data path with its transmit command header and receive
 * status word. Checksum offload, wake-on-LAN, VLAN filtering, the GPIO and
 * LED blocks and the interrupt endpoint are accepted and ignored.
 *
 * Register names, offsets and bit masks follow the Linux driver
 * (drivers/net/usb/smsc95xx.h), which is the public definition of this
 * part; the bit values below are quoted from it in the comments.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/usb.h"
#include "hw/usb/desc.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qom/object.h"

/* ------------------------------------------------------------------ */
/* Register offsets and bits (drivers/net/usb/smsc95xx.h)              */

#define SMSC_ID_REV             0x000
#define SMSC_INT_STS            0x008
#define SMSC_TX_CFG             0x010
#define  TX_CFG_ON              0x00000004
#define  TX_CFG_STOP            0x00000002
#define SMSC_HW_CFG             0x014
#define  HW_CFG_BIR             0x00001000
#define  HW_CFG_RXDOFF          0x00000600
#define  HW_CFG_MEF             0x00000020
#define  HW_CFG_LRST            0x00000008
#define  HW_CFG_PSEL            0x00000004
#define  HW_CFG_BCE             0x00000002
#define SMSC_PM_CTRL            0x020
#define  PM_CTL_PHY_RST         0x00000010
#define SMSC_E2P_CMD            0x030
#define  E2P_CMD_BUSY           0x80000000
#define  E2P_CMD_MASK           0x70000000
#define  E2P_CMD_READ           0x00000000
#define  E2P_CMD_TIMEOUT        0x00000400
#define  E2P_CMD_LOADED         0x00000200
#define  E2P_CMD_ADDR           0x000001ff
#define SMSC_E2P_DATA           0x034
#define SMSC_BURST_CAP          0x038
#define SMSC_BULK_IN_DLY        0x06c
#define SMSC_MAC_CR             0x100
#define  MAC_CR_MCPAS           0x00080000
#define  MAC_CR_PRMS            0x00040000
#define  MAC_CR_TXEN            0x00000008
#define  MAC_CR_RXEN            0x00000004
#define SMSC_ADDRH              0x104
#define SMSC_ADDRL              0x108
#define SMSC_MII_ADDR           0x114
#define  MII_WRITE              0x00000002
#define  MII_BUSY               0x00000001
#define SMSC_MII_DATA           0x118

#define SMSC_REG_WINDOW         0x200

/* Vendor control requests (smsc95xx.h) */
#define SMSC_REQ_WRITE_REGISTER 0xa0
#define SMSC_REQ_READ_REGISTER  0xa1

/*
 * Chip id reported in the top half of ID_REV. 0xec00 is the LAN9512, the
 * part on a Raspberry Pi 2. Drivers branch on 0x9e00 (LAN9500A) for an
 * errata workaround that would otherwise pull in PHY registers this model
 * does not implement.
 */
#define SMSC_CHIP_ID_9512       0xec00
#define SMSC_ID_REV_VALUE       ((SMSC_CHIP_ID_9512 << 16) | 0x0001)

/* Transmit command header (smsc95xx.h) */
#define TX_CMD_A_DATA_OFFSET    0x001f0000
#define TX_CMD_A_FIRST_SEG      0x00002000
#define TX_CMD_A_LAST_SEG       0x00001000
#define TX_CMD_B_CSUM_ENABLE    0x00004000
#define TX_CMD_B_DIS_PADDING    0x00001000
#define TX_CMD_B_FRAME_LENGTH   0x000007ff

/* Receive status word (smsc95xx.h) */
#define RX_STS_FL_SHIFT         16
#define RX_STS_FL_MASK          0x3fff

/* ------------------------------------------------------------------ */
/* PHY (internal, MDIO address 1)                                      */

#define SMSC_PHY_ADDRESS        1

#define PHY_BMCR                0
#define  BMCR_RESET             0x8000
#define  BMCR_NWAY_RESET        0x0200
#define PHY_BMSR                1
#define PHY_PHYIDR1             2
#define PHY_PHYIDR2             3
#define PHY_ANAR                4
#define PHY_ANLPAR              5

/*
 * Advertisement and link-partner words must overlap or a driver's
 * autonegotiation resolution finds no common mode and never brings the
 * link up. 0x01e1 is the conventional "all four 10/100 modes, 802.3
 * selector"; the partner word is the same plus the acknowledge bit.
 */
#define PHY_ANAR_RESET          0x01e1
#define PHY_ANLPAR_VALUE        0x41e1
#define PHY_BMCR_RESET          0x3100

/*
 * Status word. Bit 8 (extended status) is deliberately clear: setting it
 * sends drivers off to the 1000BASE-T registers, which this part does not
 * have. Neither link nor autoneg-complete latches low -- a driver polling
 * once a second must not see a spurious drop.
 */
#define PHY_BMSR_LINK_UP        0x782d
#define PHY_BMSR_LINK_DOWN      0x7829

#define SMSC_EEPROM_SIZE        512
#define SMSC_EEPROM_SIGNATURE   0xa5
#define SMSC_EEPROM_MAC_OFFSET  1

/*
 * Buffers are one BURST_CAP batch plus slack. A driver may hand us several
 * frames in one bulk OUT, and the host controller may chop one frame into
 * many packets, so both directions need to be reassembled.
 */
#define SMSC_BUF_SIZE           (16384 + 2048)

#define SMSC_MIN_FRAME          14
#define SMSC_MAX_FRAME          1514        /* without FCS */
#define SMSC_RUNT_PAD           60

#define TYPE_USB_NET_SMSC95XX "usb-net-smsc95xx"
OBJECT_DECLARE_SIMPLE_TYPE(SMSCState, USB_NET_SMSC95XX)

struct SMSCState {
    USBDevice dev;

    NICState *nic;
    NICConf conf;
    USBEndpoint *bulk_in;

    uint32_t reg[SMSC_REG_WINDOW / sizeof(uint32_t)];
    uint32_t e2p_cmd;
    uint32_t e2p_data;
    uint32_t mii_addr;
    uint32_t mii_data;
    uint8_t eeprom[SMSC_EEPROM_SIZE];

    uint16_t bmcr;
    uint16_t anar;
    uint16_t phy[32];

    uint8_t out_buf[SMSC_BUF_SIZE];
    uint32_t out_len;
    uint8_t in_buf[SMSC_BUF_SIZE];
    uint32_t in_len;
    uint32_t in_ptr;

    /*
     * A driver keeps a bulk IN outstanding the whole time it is up. Parking
     * the packet until a frame arrives, rather than answering NAK, is what
     * keeps an idle link from costing an interrupt every frame.
     */
    USBPacket *in_pkt;
    bool flush_rx;
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings smsc_stringtable = {
    [STR_MANUFACTURER] = "SMSC",
    [STR_PRODUCT]      = "LAN9512",
    [STR_SERIALNUMBER] = "1",
};

static const USBDescIface desc_iface_fs[] = {
    {
        .bInterfaceNumber              = 0,
        .bAlternateSetting             = 0,
        .bNumEndpoints                 = 3,
        .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
        .bInterfaceSubClass            = 0x00,
        .bInterfaceProtocol            = 0xff,
        .eps = (USBDescEndpoint[]) {
            {
                .bEndpointAddress      = USB_DIR_IN | 0x01,
                .bmAttributes          = USB_ENDPOINT_XFER_BULK,
                .wMaxPacketSize        = 64,
            },{
                .bEndpointAddress      = USB_DIR_OUT | 0x02,
                .bmAttributes          = USB_ENDPOINT_XFER_BULK,
                .wMaxPacketSize        = 64,
            },{
                .bEndpointAddress      = USB_DIR_IN | 0x03,
                .bmAttributes          = USB_ENDPOINT_XFER_INT,
                .wMaxPacketSize        = 16,
                .bInterval             = 4,
            },
        },
    }
};

static const USBDescIface desc_iface_hs[] = {
    {
        .bInterfaceNumber              = 0,
        .bAlternateSetting             = 0,
        .bNumEndpoints                 = 3,
        .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
        .bInterfaceSubClass            = 0x00,
        .bInterfaceProtocol            = 0xff,
        .eps = (USBDescEndpoint[]) {
            {
                .bEndpointAddress      = USB_DIR_IN | 0x01,
                .bmAttributes          = USB_ENDPOINT_XFER_BULK,
                .wMaxPacketSize        = 512,
            },{
                .bEndpointAddress      = USB_DIR_OUT | 0x02,
                .bmAttributes          = USB_ENDPOINT_XFER_BULK,
                .wMaxPacketSize        = 512,
            },{
                .bEndpointAddress      = USB_DIR_IN | 0x03,
                .bmAttributes          = USB_ENDPOINT_XFER_INT,
                .wMaxPacketSize        = 16,
                .bInterval             = 4,
            },
        },
    }
};

static const USBDescDevice desc_device_fs = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE | USB_CFG_ATT_SELFPOWER,
            .bMaxPower             = 0x32,
            .nif = ARRAY_SIZE(desc_iface_fs),
            .ifs = desc_iface_fs,
        },
    },
};

static const USBDescDevice desc_device_hs = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE | USB_CFG_ATT_SELFPOWER,
            .bMaxPower             = 0x32,
            .nif = ARRAY_SIZE(desc_iface_hs),
            .ifs = desc_iface_hs,
        },
    },
};

static const USBDesc desc_smsc95xx = {
    .id = {
        .idVendor          = 0x0424,
        .idProduct         = 0xec00,
        .bcdDevice         = 0x0200,
        .iManufacturer     = STR_MANUFACTURER,
        .iProduct          = STR_PRODUCT,
        .iSerialNumber     = STR_SERIALNUMBER,
    },
    .full = &desc_device_fs,
    .high = &desc_device_hs,
    .str  = smsc_stringtable,
};

/* ------------------------------------------------------------------ */

static bool smsc_link_up(SMSCState *s)
{
    return !qemu_get_queue(s->nic)->link_down;
}

static unsigned smsc_mps(SMSCState *s)
{
    return s->dev.speed == USB_SPEED_HIGH ? 512 : 64;
}

/*
 * Free the receive buffer. This deliberately does NOT restart the network
 * backend: draining can happen underneath receive(), and flushing there
 * would deliver the next queued frame re-entrantly, straight back into
 * this path.
 */
static void smsc_reset_in(SMSCState *s)
{
    s->in_len = 0;
    s->in_ptr = 0;
    s->flush_rx = true;
}

static void smsc_phy_reset(SMSCState *s)
{
    s->bmcr = PHY_BMCR_RESET;
    s->anar = PHY_ANAR_RESET;
    memset(s->phy, 0, sizeof(s->phy));
}

static uint16_t smsc_phy_read(SMSCState *s, unsigned phy, unsigned reg)
{
    if (phy != SMSC_PHY_ADDRESS) {
        return 0xffff;
    }
    switch (reg) {
    case PHY_BMCR:
        return s->bmcr;
    case PHY_BMSR:
        return smsc_link_up(s) ? PHY_BMSR_LINK_UP : PHY_BMSR_LINK_DOWN;
    case PHY_PHYIDR1:
        return 0x0007;
    case PHY_PHYIDR2:
        return 0xc0f1;
    case PHY_ANAR:
        return s->anar;
    case PHY_ANLPAR:
        return smsc_link_up(s) ? PHY_ANLPAR_VALUE : 0;
    default:
        return s->phy[reg & 0x1f];
    }
}

static void smsc_phy_write(SMSCState *s, unsigned phy, unsigned reg,
                           uint16_t val)
{
    if (phy != SMSC_PHY_ADDRESS) {
        return;
    }
    switch (reg) {
    case PHY_BMCR:
        if (val & BMCR_RESET) {
            /* Self-clearing: a driver polls for it to go away. */
            smsc_phy_reset(s);
            return;
        }
        /* Restart-autonegotiation is self-clearing too. */
        s->bmcr = val & ~BMCR_NWAY_RESET;
        break;
    case PHY_BMSR:
    case PHY_PHYIDR1:
    case PHY_PHYIDR2:
    case PHY_ANLPAR:
        break;                          /* read-only */
    case PHY_ANAR:
        s->anar = val;
        break;
    default:
        s->phy[reg & 0x1f] = val;
        break;
    }
}

static void smsc_lite_reset(SMSCState *s)
{
    /*
     * A soft reset clears the datapath but leaves the MAC address, the
     * EEPROM and the PHY alone.
     */
    s->reg[SMSC_HW_CFG / 4] = 0;
    s->reg[SMSC_TX_CFG / 4] = 0;
    s->reg[SMSC_MAC_CR / 4] = 0;
    s->reg[SMSC_INT_STS / 4] = 0;
    s->out_len = 0;
    s->in_len = 0;
    s->in_ptr = 0;
}

static uint32_t smsc_reg_read(SMSCState *s, unsigned off)
{
    switch (off) {
    case SMSC_ID_REV:
        return SMSC_ID_REV_VALUE;

    case SMSC_E2P_CMD:
        /*
         * The EEPROM engine completes inside the write below, so BUSY must
         * always read back clear -- a driver's poll for it has no timeout.
         * LOADED reads set, which is how a driver decides the byte it just
         * asked for is valid.
         */
        return (s->e2p_cmd & ~(E2P_CMD_BUSY | E2P_CMD_TIMEOUT)) |
               E2P_CMD_LOADED;

    case SMSC_E2P_DATA:
        return s->e2p_data & 0xff;

    case SMSC_MII_ADDR:
        /* Likewise: MDIO completes synchronously, so BUSY never reads set. */
        return s->mii_addr & ~(uint32_t)MII_BUSY;

    case SMSC_MII_DATA:
        return s->mii_data & 0xffff;

    default:
        return s->reg[off / 4];
    }
}

static void smsc_reg_write(SMSCState *s, unsigned off, uint32_t val)
{
    unsigned phy, reg;

    switch (off) {
    case SMSC_HW_CFG:
        if (val & HW_CFG_LRST) {
            smsc_lite_reset(s);
            return;                     /* LRST is self-clearing */
        }
        s->reg[off / 4] = val & ~(uint32_t)HW_CFG_LRST;
        break;

    case SMSC_PM_CTRL:
        if (val & PM_CTL_PHY_RST) {
            smsc_phy_reset(s);
            val &= ~(uint32_t)PM_CTL_PHY_RST;
        }
        s->reg[off / 4] = val;
        break;

    case SMSC_E2P_CMD:
        if (val & E2P_CMD_BUSY) {
            unsigned addr = val & E2P_CMD_ADDR;
            if ((val & E2P_CMD_MASK) == E2P_CMD_READ) {
                s->e2p_data = addr < SMSC_EEPROM_SIZE ? s->eeprom[addr] : 0xff;
            }
            /* The write and erase commands are accepted and ignored. */
        }
        s->e2p_cmd = (val & ~(E2P_CMD_BUSY | E2P_CMD_TIMEOUT)) | E2P_CMD_LOADED;
        break;

    case SMSC_E2P_DATA:
        s->e2p_data = val & 0xff;
        break;

    case SMSC_MII_ADDR:
        phy = (val >> 11) & 0x1f;
        reg = (val >> 6) & 0x1f;
        if (val & MII_BUSY) {
            if (val & MII_WRITE) {
                smsc_phy_write(s, phy, reg, s->mii_data & 0xffff);
            } else {
                s->mii_data = smsc_phy_read(s, phy, reg);
            }
        }
        s->mii_addr = val & ~(uint32_t)MII_BUSY;
        break;

    case SMSC_MII_DATA:
        s->mii_data = val & 0xffff;
        break;

    case SMSC_TX_CFG:
        if (val & TX_CFG_STOP) {
            val &= ~(uint32_t)TX_CFG_ON;
        }
        s->reg[off / 4] = val;
        break;

    case SMSC_MAC_CR:
        s->reg[off / 4] = val;
        if (val & MAC_CR_RXEN) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;

    case SMSC_BURST_CAP:
        s->reg[off / 4] = val & 0xff;
        break;

    case SMSC_INT_STS:
        s->reg[off / 4] &= ~val;        /* write one to clear */
        break;

    default:
        s->reg[off / 4] = val;
        break;
    }
}

static void smsc_handle_control(USBDevice *dev, USBPacket *p, int request,
                                int value, int index, int length,
                                uint8_t *data)
{
    SMSCState *s = USB_NET_SMSC95XX(dev);

    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) >= 0) {
        return;
    }

    switch (request) {
    case EndpointRequest | USB_REQ_GET_STATUS:
        /* Never halted; a driver checks this before clearing a stall. */
        data[0] = 0;
        data[1] = 0;
        p->actual_length = MIN(length, 2);
        break;

    case EndpointOutRequest | USB_REQ_CLEAR_FEATURE:
    case EndpointOutRequest | USB_REQ_SET_FEATURE:
        /* Endpoint halt is accepted and has nothing to do here. */
        break;

    case VendorDeviceRequest | SMSC_REQ_READ_REGISTER:
        stl_le_p(data, smsc_reg_read(s, index & (SMSC_REG_WINDOW - 1)));
        p->actual_length = MIN(length, 4);
        break;

    case VendorDeviceOutRequest | SMSC_REQ_WRITE_REGISTER:
        if (length >= 4) {
            smsc_reg_write(s, index & (SMSC_REG_WINDOW - 1), ldl_le_p(data));
        }
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "%s: unhandled request 0x%x\n", __func__,
                      request);
        p->status = USB_RET_STALL;
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Transmit                                                            */

static bool smsc_tx_enabled(SMSCState *s)
{
    return (s->reg[SMSC_TX_CFG / 4] & TX_CFG_ON) &&
           (s->reg[SMSC_MAC_CR / 4] & MAC_CR_TXEN);
}

static void smsc_bulk_out(SMSCState *s, USBPacket *p)
{
    unsigned pos = 0;
    unsigned sz = sizeof(s->out_buf) - s->out_len;

    if (sz > p->iov.size) {
        sz = p->iov.size;
    }
    usb_packet_copy(p, &s->out_buf[s->out_len], sz);
    s->out_len += sz;

    /*
     * A single transfer can carry several frames, and the host controller
     * can split one frame across several packets, so parse whatever whole
     * blocks have arrived and keep the remainder for next time.
     */
    while (s->out_len - pos >= 8) {
        uint32_t cmd_a = ldl_le_p(&s->out_buf[pos]);
        uint32_t cmd_b = ldl_le_p(&s->out_buf[pos + 4]);
        unsigned off = (cmd_a & TX_CMD_A_DATA_OFFSET) >> 16;
        unsigned len = cmd_b & TX_CMD_B_FRAME_LENGTH;
        unsigned csum = (cmd_b & TX_CMD_B_CSUM_ENABLE) ? 4 : 0;
        unsigned block;

        if (len < SMSC_MIN_FRAME || len > SMSC_MAX_FRAME + 4) {
            /* Out of step with the guest; there is no way to resynchronise. */
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: implausible frame length %u, dropping %u "
                          "buffered bytes\n", __func__, len, s->out_len);
            s->out_len = 0;
            return;
        }
        if (s->out_len - pos < 8 + off + len) {
            break;                      /* rest of the frame not here yet */
        }

        if (len > csum && smsc_tx_enabled(s)) {
            const uint8_t *frame = &s->out_buf[pos + 8 + off + csum];
            unsigned flen = len - csum;

            if (flen < SMSC_RUNT_PAD && !(cmd_b & TX_CMD_B_DIS_PADDING)) {
                uint8_t padded[SMSC_RUNT_PAD];

                memcpy(padded, frame, flen);
                memset(padded + flen, 0, SMSC_RUNT_PAD - flen);
                qemu_send_packet(qemu_get_queue(s->nic), padded,
                                 SMSC_RUNT_PAD);
            } else {
                qemu_send_packet(qemu_get_queue(s->nic), frame, flen);
            }
        }

        block = (8 + off + len + 3) & ~3u;
        pos += block;
    }

    if (pos) {
        s->out_len -= pos;
        memmove(s->out_buf, &s->out_buf[pos], s->out_len);
    }
}

/* ------------------------------------------------------------------ */
/* Receive                                                             */

static void smsc_bulk_in(SMSCState *s, USBPacket *p)
{
    unsigned mps = smsc_mps(s);
    unsigned len;

    if (s->in_ptr > s->in_len) {
        smsc_reset_in(s);
    }
    if (!s->in_len) {
        /*
         * Nothing to send yet. Hold the request open instead of answering
         * NAK: a driver polls this endpoint continuously, and a NAK is
         * retried every frame, which on an idle link is a steady stream of
         * interrupts for no data.
         */
        if (s->in_pkt) {
            p->status = USB_RET_NAK;    /* only one may be outstanding */
        } else {
            s->in_pkt = p;
            p->status = USB_RET_ASYNC;
        }
        return;
    }

    len = s->in_len - s->in_ptr;
    if (len > p->iov.size) {
        len = p->iov.size;
    }
    usb_packet_copy(p, &s->in_buf[s->in_ptr], len);
    s->in_ptr += len;

    /*
     * A short packet ends the transfer. If the batch came out an exact
     * multiple of the packet size the host is still waiting, so leave the
     * buffer in place for one more (zero length) reply before clearing it.
     */
    if (s->in_ptr >= s->in_len && ((s->in_len % mps) || !len)) {
        smsc_reset_in(s);
    }
}

static bool smsc_filter_accept(SMSCState *s, const uint8_t *buf)
{
    uint32_t mac_cr = s->reg[SMSC_MAC_CR / 4];
    uint8_t mac[6];

    if (mac_cr & MAC_CR_PRMS) {
        return true;
    }
    if (buf[0] & 0x01) {                /* group address */
        static const uint8_t bcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
        if (!memcmp(buf, bcast, sizeof(bcast))) {
            return true;
        }
        return (mac_cr & MAC_CR_MCPAS) != 0;
    }

    /*
     * Match against the address the guest programmed, not the one QEMU
     * handed out -- a driver is free to write back a different one.
     */
    stl_le_p(mac, s->reg[SMSC_ADDRL / 4]);
    stw_le_p(mac + 4, s->reg[SMSC_ADDRH / 4] & 0xffff);
    return !memcmp(buf, mac, sizeof(mac));
}

static bool smsc_can_receive(NetClientState *nc)
{
    SMSCState *s = qemu_get_nic_opaque(nc);

    return s->dev.config && (s->reg[SMSC_MAC_CR / 4] & MAC_CR_RXEN) &&
           s->in_len == 0;
}

static ssize_t smsc_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    SMSCState *s = qemu_get_nic_opaque(nc);
    unsigned block;

    if (!s->dev.config || !(s->reg[SMSC_MAC_CR / 4] & MAC_CR_RXEN)) {
        return -1;
    }
    if (size < SMSC_MIN_FRAME || size > SMSC_MAX_FRAME) {
        return -1;
    }
    if (!smsc_filter_accept(s, buf)) {
        return -1;
    }

    /* status word + frame + FCS, padded up to a word boundary */
    block = 4 + ((size + 4 + 3) & ~(size_t)3);
    if (s->in_len || block > sizeof(s->in_buf)) {
        return 0;                       /* retry once the guest has drained */
    }

    /*
     * The length in the status word counts the FCS, which is what a driver
     * expects from this part; four zero bytes stand in for it, since
     * nothing checks the CRC.
     */
    stl_le_p(&s->in_buf[0], (uint32_t)(size + 4) << RX_STS_FL_SHIFT);
    memcpy(&s->in_buf[4], buf, size);
    memset(&s->in_buf[4 + size], 0, block - 4 - size);
    s->in_len = block;
    s->in_ptr = 0;

    if (s->in_pkt) {
        USBPacket *p = s->in_pkt;

        s->in_pkt = NULL;
        p->status = USB_RET_SUCCESS;
        smsc_bulk_in(s, p);
        usb_packet_complete(USB_DEVICE(s), p);
    } else {
        usb_wakeup(s->bulk_in, 0);
    }
    return size;
}

static void smsc_handle_data(USBDevice *dev, USBPacket *p)
{
    SMSCState *s = USB_NET_SMSC95XX(dev);

    switch (p->pid) {
    case USB_TOKEN_OUT:
        if (p->ep->nr == 2) {
            smsc_bulk_out(s, p);
        } else {
            goto fail;
        }
        break;

    case USB_TOKEN_IN:
        if (p->ep->nr == 1) {
            smsc_bulk_in(s, p);
            /*
             * Safe to restart the backend here, but never from receive():
             * see smsc_reset_in().
             */
            if (s->flush_rx) {
                s->flush_rx = false;
                qemu_flush_queued_packets(qemu_get_queue(s->nic));
            }
        } else if (p->ep->nr == 3) {
            /* Interrupt endpoint: no status is ever raised. */
            p->status = USB_RET_NAK;
        } else {
            goto fail;
        }
        break;

    default:
    fail:
        p->status = USB_RET_STALL;
        break;
    }
}

static void smsc_cancel_packet(USBDevice *dev, USBPacket *p)
{
    SMSCState *s = USB_NET_SMSC95XX(dev);

    if (s->in_pkt == p) {
        s->in_pkt = NULL;
    }
}

static void smsc_handle_reset(USBDevice *dev)
{
    SMSCState *s = USB_NET_SMSC95XX(dev);

    s->in_pkt = NULL;
    s->flush_rx = false;
    memset(s->reg, 0, sizeof(s->reg));
    s->e2p_cmd = E2P_CMD_LOADED;
    s->e2p_data = 0;
    s->mii_addr = 0;
    s->mii_data = 0;
    s->out_len = 0;
    s->in_len = 0;
    s->in_ptr = 0;
    smsc_phy_reset(s);

    /*
     * Seed the address registers so a guest that skips the EEPROM still
     * finds the address QEMU assigned.
     */
    s->reg[SMSC_ADDRL / 4] = ldl_le_p(s->conf.macaddr.a);
    s->reg[SMSC_ADDRH / 4] = lduw_le_p(s->conf.macaddr.a + 4);
}

static void smsc_cleanup(NetClientState *nc)
{
    SMSCState *s = qemu_get_nic_opaque(nc);

    s->nic = NULL;
}

static NetClientInfo net_smsc95xx_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = smsc_can_receive,
    .receive = smsc_receive,
    .cleanup = smsc_cleanup,
};

static void smsc_realize(USBDevice *dev, Error **errp)
{
    SMSCState *s = USB_NET_SMSC95XX(dev);

    usb_desc_create_serial(dev);
    usb_desc_init(dev);

    s->bulk_in = usb_ep_get(dev, USB_TOKEN_IN, 1);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);

    /*
     * Publish the MAC through a small EEPROM image. A driver that finds the
     * signature byte adopts the address from it, which keeps the guest and
     * QEMU agreeing on what this interface is.
     */
    memset(s->eeprom, 0xff, sizeof(s->eeprom));
    s->eeprom[0] = SMSC_EEPROM_SIGNATURE;
    memcpy(&s->eeprom[SMSC_EEPROM_MAC_OFFSET], s->conf.macaddr.a,
           sizeof(s->conf.macaddr.a));

    s->nic = qemu_new_nic(&net_smsc95xx_info, &s->conf,
                          object_get_typename(OBJECT(s)), dev->qdev.id,
                          &dev->qdev.mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);

    smsc_handle_reset(dev);
}

static void smsc_unrealize(USBDevice *dev)
{
    SMSCState *s = USB_NET_SMSC95XX(dev);

    qemu_del_nic(s->nic);
}

static const VMStateDescription vmstate_smsc95xx = {
    .name = TYPE_USB_NET_SMSC95XX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_USB_DEVICE(dev, SMSCState),
        VMSTATE_UINT32_ARRAY(reg, SMSCState,
                             SMSC_REG_WINDOW / sizeof(uint32_t)),
        VMSTATE_UINT32(e2p_cmd, SMSCState),
        VMSTATE_UINT32(e2p_data, SMSCState),
        VMSTATE_UINT32(mii_addr, SMSCState),
        VMSTATE_UINT32(mii_data, SMSCState),
        VMSTATE_UINT16(bmcr, SMSCState),
        VMSTATE_UINT16(anar, SMSCState),
        VMSTATE_UINT16_ARRAY(phy, SMSCState, 32),
        VMSTATE_UINT8_ARRAY(out_buf, SMSCState, SMSC_BUF_SIZE),
        VMSTATE_UINT32(out_len, SMSCState),
        VMSTATE_UINT8_ARRAY(in_buf, SMSCState, SMSC_BUF_SIZE),
        VMSTATE_UINT32(in_len, SMSCState),
        VMSTATE_UINT32(in_ptr, SMSCState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property smsc_properties[] = {
    DEFINE_NIC_PROPERTIES(SMSCState, conf),
};

static void smsc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

    uc->realize        = smsc_realize;
    uc->unrealize      = smsc_unrealize;
    uc->product_desc   = "SMSC LAN9512 USB 2.0 Ethernet";
    uc->usb_desc       = &desc_smsc95xx;
    uc->handle_reset   = smsc_handle_reset;
    uc->handle_control = smsc_handle_control;
    uc->handle_data    = smsc_handle_data;
    uc->cancel_packet  = smsc_cancel_packet;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->fw_name = "ethernet";
    dc->vmsd = &vmstate_smsc95xx;
    device_class_set_props(dc, smsc_properties);
}

static const TypeInfo smsc_info = {
    .name          = TYPE_USB_NET_SMSC95XX,
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(SMSCState),
    .class_init    = smsc_class_init,
};

static void smsc_register_types(void)
{
    type_register_static(&smsc_info);
}

type_init(smsc_register_types)
