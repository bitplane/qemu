/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "libqtest.h"

#define HFNUM 0x3f980408
#define HPRT0 0x3f980440
#define PORT_RESET (1U << 8)
#define PORT_ENABLE (1U << 2)

static void reset_port(QTestState *qts)
{
    qtest_writel(qts, HPRT0, PORT_RESET);
    qtest_writel(qts, HPRT0, 0);
    g_assert_cmphex(qtest_readl(qts, HPRT0) & PORT_ENABLE, ==, PORT_ENABLE);
}

static void test_port_enable(void)
{
    QTestState *qts = qtest_init("-machine raspi2b -device usb-kbd");
    unsigned frame = qtest_readl(qts, HFNUM) & 0x3fff;

    qtest_clock_step(qts, 1000000);
    g_assert_cmpuint(qtest_readl(qts, HFNUM) & 0x3fff, ==, frame);
    for (unsigned i = 0; i < 2; i++) {
        reset_port(qts);
        frame = qtest_readl(qts, HFNUM) & 0x3fff;
        qtest_clock_step(qts, 1000000);
        g_assert_cmpuint(qtest_readl(qts, HFNUM) & 0x3fff, !=, frame);
    }
    qtest_writel(qts, HPRT0, PORT_ENABLE);
    g_assert_cmphex(qtest_readl(qts, HPRT0) & PORT_ENABLE, ==, 0);
    frame = qtest_readl(qts, HFNUM) & 0x3fff;
    qtest_clock_step(qts, 1000000);
    g_assert_cmpuint(qtest_readl(qts, HFNUM) & 0x3fff, ==, frame);
    reset_port(qts);
    frame = qtest_readl(qts, HFNUM) & 0x3fff;
    qtest_clock_step(qts, 1000000);
    g_assert_cmpuint(qtest_readl(qts, HFNUM) & 0x3fff, !=, frame);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/dwc2/port-enable", test_port_enable);
    return g_test_run();
}
