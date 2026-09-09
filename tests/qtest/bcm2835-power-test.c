/*
 * BCM2835 legacy power mailbox tests.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define MAILBOX_READ 0x3f00b880
#define MAILBOX_STATUS 0x3f00b898
#define MAILBOX_WRITE 0x3f00b8a0
#define MAILBOX_EMPTY (1U << 30)

static void test_power(void)
{
    QTestState *qts = qtest_init("-machine raspi2b");
    const uint32_t requests[] = { 0x80, 0x1ff0, 0, 0x10 };

    for (unsigned i = 0; i < ARRAY_SIZE(requests); i++) {
        qtest_writel(qts, MAILBOX_WRITE, requests[i]);
        g_assert_cmphex(qtest_readl(qts, MAILBOX_STATUS) & MAILBOX_EMPTY,
                        ==, 0);
        g_assert_cmphex(qtest_readl(qts, MAILBOX_READ), ==, requests[i]);
        g_assert_cmphex(qtest_readl(qts, MAILBOX_STATUS) & MAILBOX_EMPTY,
                        ==, MAILBOX_EMPTY);
    }

    qtest_writel(qts, MAILBOX_WRITE, 0x80);
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, MAILBOX_STATUS) & MAILBOX_EMPTY,
                    ==, MAILBOX_EMPTY);
    qtest_writel(qts, MAILBOX_WRITE, 0x10);
    g_assert_cmphex(qtest_readl(qts, MAILBOX_READ), ==, 0x10);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/bcm2835/power-mailbox", test_power);
    return g_test_run();
}
