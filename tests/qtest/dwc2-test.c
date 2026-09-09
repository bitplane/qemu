/*
 * DWC2 host controller tests.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DWC2_BASE 0x3f980000
#define GINTSTS (DWC2_BASE + 0x014)
#define HAINT (DWC2_BASE + 0x414)
#define HAINTMSK (DWC2_BASE + 0x418)
#define HCCHAR0 (DWC2_BASE + 0x500)
#define HCINTMSK0 (DWC2_BASE + 0x50c)
#define HCHINT (1U << 25)
#define CHHLTD (1U << 1)
#define CHDIS (1U << 30)

static void test_interrupt_mask(void)
{
    QTestState *qts = qtest_init("-machine raspi2b");

    qtest_writel(qts, HCINTMSK0, CHHLTD);
    qtest_writel(qts, HCCHAR0, CHDIS);
    g_assert_cmphex(qtest_readl(qts, HAINT), ==, 1);
    g_assert_cmphex(qtest_readl(qts, GINTSTS) & HCHINT, ==, 0);

    qtest_writel(qts, HAINTMSK, 1);
    g_assert_cmphex(qtest_readl(qts, GINTSTS) & HCHINT, ==, HCHINT);
    qtest_writel(qts, HAINTMSK, 0);
    g_assert_cmphex(qtest_readl(qts, GINTSTS) & HCHINT, ==, 0);
    g_assert_cmphex(qtest_readl(qts, HAINT), ==, 1);
    qtest_writel(qts, HAINTMSK, 1);
    g_assert_cmphex(qtest_readl(qts, GINTSTS) & HCHINT, ==, HCHINT);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/dwc2/interrupt-mask", test_interrupt_mask);
    return g_test_run();
}
