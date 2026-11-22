/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define CMU 0x40000000U
#define IOMUX 0x40086000U
#define GPIO0 0x40081000U
#define GPIO2 0x4008a000U
#define I2C3_PCLK BIT(22)
#define I2C3_OCLK BIT(24)

static void configure(uintptr_t reg, uint32_t mask, uint32_t value)
{
	sys_write32((sys_read32(reg) & ~mask) | value, reg);
}

static void configure_touch_pins(void)
{
	/* Keep reset inactive until the controller driver takes ownership. */
	sys_write32(BIT(28), GPIO0 + 0x00U);
	sys_write32(BIT(28), GPIO0 + 0x04U);
	sys_write32(BIT(27), GPIO0 + 0x10U);
	configure(IOMUX + 0x10U, GENMASK(19, 12), 0);
	configure(IOMUX + 0x2cU, BIT(27) | BIT(28), BIT(27));
	configure(IOMUX + 0x30U, BIT(27) | BIT(28), 0);
}

static void configure_i2c3(void)
{
	/* I2C3 is on GPIO6/7 and uses the single crystal source. */
	configure(IOMUX + 0x04U, GENMASK(31, 24), 0x11000000U);
	configure(IOMUX + 0x50U, BIT(13), BIT(13));
	configure(IOMUX + 0x2cU, BIT(6) | BIT(7), 0);
	configure(IOMUX + 0x30U, BIT(6) | BIT(7), 0);
	configure(CMU + 0x74U, GENMASK(5, 3), 0);
	sys_write32(I2C3_PCLK, CMU + 0x08U);
	sys_write32(I2C3_OCLK, CMU + 0x10U);
	sys_write32(I2C3_PCLK, CMU + 0x40U);
	sys_write32(I2C3_OCLK, CMU + 0x48U);
}

static void enable_panel_touch_power(void)
{
	/* GPIO72 through GPIO75 hold the shared panel and touch supply state. */
	sys_write32(GENMASK(11, 8), GPIO2 + 0x00U);
	sys_write32(GENMASK(11, 8), GPIO2 + 0x04U);
	configure(IOMUX + 0x90U, GENMASK(15, 0), 0);
	configure(IOMUX + 0x9cU, BIT(26) | BIT(10), 0);
}

static int board_io_init(void)
{
	if (IS_ENABLED(CONFIG_INPUT_ZTW623) || IS_ENABLED(CONFIG_BES2700IMP_PANEL)) {
		enable_panel_touch_power();
	}
	if (IS_ENABLED(CONFIG_INPUT_ZTW623)) {
		configure_touch_pins();
	}
	if (IS_ENABLED(CONFIG_I2C_DW) && DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(i2c3))) {
		configure_i2c3();
	}
	return 0;
}

SYS_INIT(board_io_init, PRE_KERNEL_1, 39);
