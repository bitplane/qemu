// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM2835 mailbox channel 0 (VideoCore power management)
 */

#ifndef BCM2835_MBOX_POWER_H
#define BCM2835_MBOX_POWER_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_BCM2835_MBOX_POWER "bcm2835-mbox-power"
OBJECT_DECLARE_SIMPLE_TYPE(BCM2835MboxPowerState, BCM2835_MBOX_POWER)

struct BCM2835MboxPowerState {
    /*< private >*/
    SysBusDevice busdev;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq mbox_irq;

    uint32_t state;             /* current power state bitmask */
    uint32_t reply;             /* queued mailbox response word */
    bool pending;
};

#endif
