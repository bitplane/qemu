// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM2835 VCHIQ peer (mailbox channel 3 + the VCHIQ doorbells)
 */

#ifndef BCM2835_MBOX_VCHIQ_H
#define BCM2835_MBOX_VCHIQ_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_BCM2835_MBOX_VCHIQ "bcm2835-mbox-vchiq"
OBJECT_DECLARE_SIMPLE_TYPE(BCM2835MboxVchiqState, BCM2835_MBOX_VCHIQ)

struct BCM2835MboxVchiqState {
    /*< private >*/
    SysBusDevice busdev;
    /*< public >*/

    MemoryRegion iomem;         /* mailbox child window (channel 3) */
    MemoryRegion dbell;         /* the four doorbell registers */
    MemoryRegion *dma_mr;
    AddressSpace dma_as;
    qemu_irq irq;               /* ARM basic bank, doorbell 0 */

    bool active;                /* slot zero accepted, CONNECT delivered */
    uint32_t slot_zero;         /* bus address the guest handed over */
    uint32_t master;            /* bus address of the master shared state */
    uint32_t slave;             /* bus address of the slave shared state */
    uint32_t slot_size;
    uint32_t max_slots_per_side;
    uint32_t slot_first;        /* first slot owned by the master */
    uint32_t slot_last;         /* last slot owned by the master */
    uint32_t tx_pos;            /* master's write position */
    uint32_t rx_pos;            /* our read position in the slave's stream */
    uint32_t doorbell0;         /* pending VC -> ARM doorbell bits */
};

#endif
