/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "libqtest.h"

#define HFNUM 0x3f980408
#define HPRT0 0x3f980440
#define PORT_RESET (1U << 8)

static void test_frame_timing(void)
{
    QTestState *qts = qtest_init("-machine raspi2b -device usb-kbd");
    unsigned start;

    qtest_writel(qts, HPRT0, PORT_RESET);
    qtest_writel(qts, HPRT0, 0);
    qtest_clock_step(qts, 1000000);
    start = qtest_readl(qts, HFNUM) & 0x3fff;
    qtest_clock_step(qts, 1000000);
    g_assert_cmpuint(qtest_readl(qts, HFNUM) & 0x3fff, ==,
                     (start + 1) & 0x3fff);
    qtest_clock_step(qts, 10500000);
    g_assert_cmpuint(qtest_readl(qts, HFNUM) & 0x3fff, ==,
                     (start + 11) & 0x3fff);
    qtest_clock_step(qts, 500000);
    g_assert_cmpuint(qtest_readl(qts, HFNUM) & 0x3fff, ==,
                     (start + 12) & 0x3fff);
    qtest_clock_step(qts, 65536000000LL);
    g_assert_cmpuint(qtest_readl(qts, HFNUM) & 0x3fff, ==,
                     (start + 12) & 0x3fff);
    qtest_clock_step(qts, 1000000);
    g_assert_cmpuint(qtest_readl(qts, HFNUM) & 0x3fff, ==,
                     (start + 13) & 0x3fff);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/dwc2/frame-timing", test_frame_timing);
    return g_test_run();
}
