/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

#include "bes2700imp_lcdc.h"

#define GPIO_SET       0x00U
#define GPIO_DIR_SET   0x04U
#define GPIO_CLR       0x0cU
#define GPIO_DIR_CLR   0x10U
#define GPIO_LEVEL     0x50U
#define COMMAND_DELAY_US 5U

#define LCDC_GRA_ADDRESS 0x0f4U
#define LCDC_GRA_PITCH   0x0fcU
#define LCDC_GRA_SIZE    0x104U
#define LCDC_GRA_ZOOM    0x108U
#define LCDC_PANEL_TOTAL 0x114U
#define LCDC_PANEL_SIZE  0x118U
#define LCDC_IRQ_STATUS  0x1c4U
#define LCDC_FRAME_CTRL  0x210U
#define LCDC_TRANSFER    0x224U
#define LCDC_CTRL_230    0x230U
#define LCDC_GRA_CTRL    0x264U
#define LCDC_GENERATE    BIT(29)
#define LCDC_START       BIT(0)
#define LCDC_TXC         BIT(21)
#define LCDC_ERRORS      0x22010000U
#define LCDC_MUX_MASK    GENMASK(23, 0)
#define LCDC_MUX_QUAD    0x222202U
#define LCDC_GUARD_WORD  0x51a7U
#define LCDC_SPI_SCLK_COUNT 2U

#define CMU 0x40000000U
#define AON_CMU 0x40080000U
#define PSC_DISPLAY 0x40085028U
#define CMU_HCLK_ENABLE (CMU + 0x00U)
#define CMU_PCLK_ENABLE (CMU + 0x08U)
#define CMU_OCLK_ENABLE (CMU + 0x10U)
#define CMU_PCLK_MODE (CMU + 0x1cU)
#define CMU_HRESET_SET (CMU + 0x34U)
#define CMU_PRESET_CLEAR (CMU + 0x40U)
#define CMU_ORESET_SET (CMU + 0x44U)
#define CMU_SYS_CLOCK (CMU + 0x60U)
#define CMU_DSI_ENABLE (CMU + 0x100U)
#define CMU_DSI_DISABLE (CMU + 0x104U)
#define CMU_XCLK_ENABLE (CMU + 0x140U)
#define CMU_XCLK_MODE (CMU + 0x148U)
#define CMU_XRESET_SET (CMU + 0x150U)
#define CMU_XRESET_CLEAR (CMU + 0x154U)
#define CMU_DISPLAY_GATE_ENABLE (CMU + 0x168U)
#define CMU_DISPLAY_GATE_DISABLE (CMU + 0x16cU)
#define CMU_DISPLAY_GATE_RESET_SET (CMU + 0x174U)
#define CMU_DISPLAY_GATE_RESET_CLEAR (CMU + 0x178U)
#define CPU_SOURCE_MASK GENMASK(12, 5)
#define OSC_MULTIPLIERS_POWER (BIT(16) | BIT(14))
#define OSC_MULTIPLIERS_ENABLE (BIT(15) | BIT(13))
#define CMU_DISPLAY_SELECTORS GENMASK(17, 8)
#define CMU_PIX_SELECT_OSCX2 BIT(10)
#define CMU_DISPLAY_SELECT_OSCX4 BIT(14)
#define DISPLAY_CLOCK_SELECT (CMU_PIX_SELECT_OSCX2 | CMU_DISPLAY_SELECT_OSCX4)
#define LCDC_HCLK BIT(21)
#define DISPLAY_OCLKS 0xb8200000U
#define DISPLAY_PCLK BIT(19)
#define DISPLAY_XCLKS GENMASK(8, 3)
#define DISPLAY_XCLK_MANUAL (BIT(8) | BIT(7))
#define DISPLAY_GATE BIT(10)
#define DISPLAY_GATE_GROUP GENMASK(10, 0)
#define LCDC_PADS_DRIVE 0x400860a0U
#define LCDC_PADS_CONTROL 0x400860acU

static uint32_t data_mask(const struct bes2700imp_lcdc *cfg)
{
	return 0xfU << cfg->d0;
}

static bool bus_idle(const struct bes2700imp_lcdc *cfg)
{
	uint32_t controls = cfg->clk | cfg->cs;
	uint32_t pins = controls | data_mask(cfg);
	uint32_t expected = cfg->cs;

	return (sys_read32(cfg->mux) & GENMASK(23, 0)) == 0U &&
	       (sys_read32(cfg->gpio + GPIO_DIR_SET) & pins) == pins &&
	       (sys_read32(cfg->gpio + GPIO_SET) & controls) == expected &&
	       (sys_read32(cfg->gpio + GPIO_LEVEL) & controls) == expected &&
	       (sys_read32(cfg->reset_mux) & GENMASK(3, 0)) == 0U &&
	       (sys_read32(cfg->reset_gpio + GPIO_DIR_SET) &
		sys_read32(cfg->reset_gpio + GPIO_LEVEL) & cfg->reset) != 0U;
}

static void clock_bit(const struct bes2700imp_lcdc *cfg)
{
	k_busy_wait(COMMAND_DELAY_US);
	sys_write32(cfg->clk, cfg->gpio + GPIO_SET);
	k_busy_wait(COMMAND_DELAY_US);
}

static void send_byte(const struct bes2700imp_lcdc *cfg, uint8_t byte)
{
	uint32_t d0 = BIT(cfg->d0);

	for (unsigned int bit = 0; bit < 8; bit++, byte <<= 1) {
		sys_write32(d0, cfg->gpio + ((byte & 0x80U) ? GPIO_SET : GPIO_CLR));
		clock_bit(cfg);
		sys_write32(cfg->clk, cfg->gpio + GPIO_CLR);
	}
}

static int select_panel(const struct bes2700imp_lcdc *cfg, uint8_t op, uint8_t cmd)
{
	if (!bus_idle(cfg)) {
		return -EIO;
	}
	sys_write32(cfg->cs, cfg->gpio + GPIO_CLR);
	k_busy_wait(COMMAND_DELAY_US);
	send_byte(cfg, op);
	send_byte(cfg, 0);
	send_byte(cfg, cmd);
	send_byte(cfg, 0);
	return 0;
}

static int deselect_panel(const struct bes2700imp_lcdc *cfg)
{
	k_busy_wait(COMMAND_DELAY_US);
	sys_write32(cfg->cs, cfg->gpio + GPIO_SET);
	k_busy_wait(COMMAND_DELAY_US);
	sys_write32(cfg->clk | data_mask(cfg), cfg->gpio + GPIO_CLR);
	return bus_idle(cfg) ? 0 : -EIO;
}

int bes2700imp_lcdc_command(const struct bes2700imp_lcdc *cfg, uint8_t cmd, const uint8_t *buf,
			  size_t size)
{
	int rc = select_panel(cfg, 0x02U, cmd);

	if (rc < 0) {
		return rc;
	}
	for (size_t i = 0; i < size; i++) {
		send_byte(cfg, buf[i]);
	}
	return deselect_panel(cfg);
}

int bes2700imp_lcdc_read(const struct bes2700imp_lcdc *cfg, uint8_t cmd, uint8_t *value)
{
	uint32_t d0 = BIT(cfg->d0);
	int rc = select_panel(cfg, 0x03U, cmd);

	if (rc < 0) {
		return rc;
	}
	*value = 0U;
	sys_write32(d0, cfg->gpio + GPIO_DIR_CLR);
	if ((sys_read32(cfg->gpio + GPIO_DIR_SET) & d0) == 0U) {
		for (unsigned int bit = 0; bit < 8; bit++) {
			clock_bit(cfg);
			*value = (*value << 1) | !!(sys_read32(cfg->gpio + GPIO_LEVEL) & d0);
			sys_write32(cfg->clk, cfg->gpio + GPIO_CLR);
		}
	} else {
		rc = -EIO;
	}
	sys_write32(cfg->cs, cfg->gpio + GPIO_SET);
	sys_write32(d0, cfg->gpio + GPIO_CLR);
	sys_write32(d0, cfg->gpio + GPIO_DIR_SET);
	return deselect_panel(cfg) == 0 ? rc : -EIO;
}

static void display_clock_output_enable(void)
{
	sys_write32(0xcaf5083fU, PSC_DISPLAY);
	k_busy_wait(20);
	sys_write32(0xcaf50837U, PSC_DISPLAY);
	k_busy_wait(20);
	sys_write32(0xcaf50830U, PSC_DISPLAY);
}

static void reset_pulse(uintptr_t set, uint32_t mask)
{
	sys_write32(mask, set);
	k_busy_wait(2);
	sys_write32(mask, set + 4U);
	k_busy_wait(2);
}

static int configure_display_clocks(void)
{
	/* Enable the display reset and clock domains before programming the controller.
	 */
	sys_write32(0xcafe02ffU, PSC_DISPLAY);
	k_busy_wait(1000);
	sys_write32(0xcafe02f7U, PSC_DISPLAY);
	k_busy_wait(1000);
	sys_write32(0xcafe02f0U, PSC_DISPLAY);
	k_busy_wait(1000);

	/* Shut down the display clock group before releasing the LCDC path.
	 * The other CMU clock groups are managed by their respective users.
	 */
	sys_write32(DISPLAY_GATE_GROUP, CMU_DISPLAY_GATE_RESET_SET);
	sys_write32(DISPLAY_GATE_GROUP, CMU_DISPLAY_GATE_DISABLE);
	k_busy_wait(2);
	const uint32_t clocks_off = sys_read32(CMU_DISPLAY_GATE_ENABLE);
	const uint32_t released_before = sys_read32(CMU_DISPLAY_GATE_RESET_CLEAR);

	display_clock_output_enable();
	sys_write32(LCDC_HCLK, CMU_HCLK_ENABLE);
	sys_write32(DISPLAY_PCLK, CMU_PCLK_ENABLE);
	/* The stock display helper enables OCLK bits 31, 29, 28, 27 and 21. */
	sys_write32(DISPLAY_OCLKS, CMU_OCLK_ENABLE);
	sys_write32(DISPLAY_XCLKS, CMU_XCLK_ENABLE);
	sys_set_bits(CMU_PCLK_MODE, DISPLAY_PCLK);
	sys_set_bits(CMU_XCLK_MODE, DISPLAY_XCLK_MANUAL);

	sys_write32(OSC_MULTIPLIERS_POWER, CMU_SYS_CLOCK);
	k_busy_wait(30);
	sys_write32(OSC_MULTIPLIERS_ENABLE, CMU_SYS_CLOCK);
	sys_clear_bits(AON_CMU + 0x18U, BIT(11));
	/* Select OSCx2 for PIX and OSCx4 for the display tap. Both registers are
	 * write-one commands, so the writes cover only the display selectors.
	 */
	sys_write32(CMU_DISPLAY_SELECTORS & ~DISPLAY_CLOCK_SELECT, CMU_DSI_DISABLE);
	sys_write32(DISPLAY_CLOCK_SELECT, CMU_DSI_ENABLE);
	display_clock_output_enable();
	sys_write32(DISPLAY_GATE, CMU_DISPLAY_GATE_ENABLE);
	const uint32_t dsi = sys_read32(CMU_DSI_ENABLE);
	const uint32_t xclk = sys_read32(CMU_XCLK_ENABLE);
	const uint32_t display = sys_read32(CMU_DISPLAY_GATE_ENABLE);

	if ((dsi & CMU_DISPLAY_SELECTORS) != DISPLAY_CLOCK_SELECT ||
	    (xclk & DISPLAY_XCLKS) != DISPLAY_XCLKS ||
	    ((clocks_off | released_before) & DISPLAY_GATE_GROUP) != 0U ||
	    (display & DISPLAY_GATE_GROUP) != DISPLAY_GATE) {
		return -EIO;
	}

	/* The display domains now have their functional clocks. Release and
	 * pulse the resets in the same order as the controller setup.
	 */
	sys_write32(DISPLAY_PCLK, CMU_PRESET_CLEAR);
	sys_write32(DISPLAY_XCLKS, CMU_XRESET_CLEAR);
	reset_pulse(CMU_ORESET_SET, BIT(29));
	reset_pulse(CMU_XRESET_SET, BIT(6));
	reset_pulse(CMU_XRESET_SET, BIT(8));
	reset_pulse(CMU_HRESET_SET, LCDC_HCLK);
	reset_pulse(CMU_XRESET_SET, BIT(7));
	reset_pulse(CMU_ORESET_SET, BIT(28));
	sys_write32(BIT(21), CMU_ORESET_SET + 4U);
	sys_write32(DISPLAY_GATE, CMU_DISPLAY_GATE_RESET_CLEAR);
	reset_pulse(CMU_ORESET_SET, BIT(27));
	reset_pulse(CMU_ORESET_SET, BIT(28));
	const uint32_t released_after = sys_read32(CMU_DISPLAY_GATE_RESET_CLEAR);

	return (released_after & DISPLAY_GATE_GROUP) == DISPLAY_GATE ? 0 : -EIO;
}

static void configure_frame_path(uintptr_t lcdc, uint32_t dimensions)
{
	/* Power-on values for the GRA/SMPN path observed before the D01 frame.
	 * Display write configures the surface and triggers each transfer.
	 */
	const struct {
		uint16_t offset;
		uint32_t value;
	} registers[] = {
		{0x21c, 0x00020020}, {0x220, 0x001f001e}, /* porch and sync */
		{0x11c, 0x000c000a}, {0x120, 0x000a000a},
		{0x100, 0x80000000}, {0x0e8, 0x80000000}, /* GRA and DMA starts */
		{0x0f0, dimensions},                /* video layer zoom */
		{0x124, 0},                               /* background */
		{0x1dc, 0x00000f00}, {0x200, 0x00000800}, /* burst fields */
		{0x1ec, 0x00000012}, {0x214, 5}, {0x228, 0x4a000000},
		{0x2a8, 0x10009f01}, {0x1c8, 0x00040000}, {0x254, 0x203},
		{0x1c0, 0x00200000}, {LCDC_IRQ_STATUS, 0},
		{0x1f8, 0x43}, {0x22c, 0x86002c00}, {0x188, 0x6501},
		{0x180, 0x02000f88}, {LCDC_FRAME_CTRL, 0x0c000003}, {LCDC_TRANSFER, 0},
	};

	sys_write32((sys_read32(LCDC_PADS_DRIVE) & ~GENMASK(27, 16)) | 0x0aaa0000U,
		    LCDC_PADS_DRIVE);
	sys_set_bits(LCDC_PADS_CONTROL, BIT(9));
	sys_clear_bits(lcdc + 0x1e8U, GENMASK(1, 0));
	sys_clear_bits(lcdc + 0x260U, BIT(27));
	for (size_t i = 0U; i < ARRAY_SIZE(registers); i++) {
		sys_write32(registers[i].value, lcdc + registers[i].offset);
	}
}

static bool lcdc_guard_ok(const struct bes2700imp_lcdc *cfg)
{
	uintptr_t end = cfg->lcdc_buffer + cfg->lcdc_buffer_size;

	for (uintptr_t p = end - BES2700IMP_LCDC_GUARD_SIZE; p < end; p += sizeof(uint16_t)) {
		if (sys_read16(p) != LCDC_GUARD_WORD) {
			return false;
		}
	}
	return true;
}

static bool lcdc_set(uintptr_t reg, uint32_t val, uint32_t stable)
{
	sys_write32(val, reg);
	return ((sys_read32(reg) ^ val) & stable) == 0U;
}

static bool lcdc_configure(const struct bes2700imp_lcdc *cfg, uint32_t width, uint32_t height,
			   uint32_t pitch)
{
	/* RGB565 quad transfer profile verified with full frames at the boot PLL rate. */
	static const struct {
		uint16_t reg;
		uint32_t val;
		uint32_t stable;
	} profile[] = {
		{LCDC_GRA_CTRL, 0, UINT32_MAX},
		{0x1a8, 128, UINT32_MAX},
		{0x1f8, 0x43, UINT32_MAX},
		{0x180, (LCDC_SPI_SCLK_COUNT << 24) | 0x00000f89U, 0xfffffffe},
		{0x22c, 0x86002c00, UINT32_MAX},
		{0x188, 0x6501, UINT32_MAX},
		{0x194, 0x80027f81, 0x7fffffff},
	};
	uintptr_t lcd = cfg->lcdc;
	uint32_t size = width | (height << 16);
	/* The timing deltas are those of the already initialized panel. */
	uint32_t total = (sys_read32(lcd + LCDC_PANEL_TOTAL) & 0xf000f000U) |
			 (width + 31U) | ((height + 21U) << 16);

	for (size_t i = 0; i < ARRAY_SIZE(profile); i++) {
		if (!lcdc_set(lcd + profile[i].reg, profile[i].val, profile[i].stable)) {
			return false;
		}
	}
	return lcdc_set(lcd + LCDC_GRA_ADDRESS, cfg->lcdc_buffer, UINT32_MAX) &&
	       lcdc_set(lcd + LCDC_GRA_PITCH, pitch, UINT32_MAX) &&
	       lcdc_set(lcd + LCDC_GRA_SIZE, size, UINT32_MAX) &&
	       lcdc_set(lcd + LCDC_GRA_ZOOM, size, UINT32_MAX) &&
	       lcdc_set(lcd + LCDC_PANEL_TOTAL, total, UINT32_MAX) &&
	       lcdc_set(lcd + LCDC_PANEL_SIZE, size, UINT32_MAX) &&
	       lcdc_set(lcd + LCDC_GRA_CTRL, 1U, UINT32_MAX);
}


static int lcdc_transfer(const struct bes2700imp_lcdc *cfg)
{
	const uintptr_t lcd = cfg->lcdc;
	const uint32_t mux = sys_read32(cfg->mux);
	const uint32_t frame = sys_read32(lcd + LCDC_FRAME_CTRL);
	const uint32_t transfer = sys_read32(lcd + LCDC_TRANSFER);
	uint32_t status;
	int ret = -EIO;

	sys_clear_bits(lcd + LCDC_CTRL_230, BIT(6));
	sys_write32(0, lcd + LCDC_IRQ_STATUS);
	if ((sys_read32(lcd + LCDC_IRQ_STATUS) & (LCDC_TXC | LCDC_ERRORS)) != 0U) {
		return -EIO;
	}

	if (lcdc_set(cfg->mux, (mux & ~LCDC_MUX_MASK) | LCDC_MUX_QUAD, LCDC_MUX_MASK)) {
		sys_write32(cfg->cs, cfg->gpio + GPIO_CLR);
		k_busy_wait(COMMAND_DELAY_US);
		if (((sys_read32(cfg->gpio + GPIO_SET) | sys_read32(cfg->gpio + GPIO_LEVEL)) &
		     cfg->cs) == 0U) {
			sys_write32(frame | LCDC_GENERATE, lcd + LCDC_FRAME_CTRL);
			status = sys_read32(lcd + LCDC_IRQ_STATUS);
			if ((status & LCDC_ERRORS) == 0U) {
				uint32_t start = k_cycle_get_32();

				sys_write32(transfer | LCDC_START, lcd + LCDC_TRANSFER);
				for (unsigned int budget = 1000000U; budget > 0U; budget--) {
					status = sys_read32(lcd + LCDC_IRQ_STATUS);
					if ((status & (LCDC_TXC | LCDC_ERRORS)) != 0U ||
					    k_cycle_get_32() - start >=
						sys_clock_hw_cycles_per_sec() / 10U) {
						break;
					}
				}
				ret = (status & LCDC_ERRORS) != 0U ? -EIO :
				      (status & LCDC_TXC) != 0U ? 0 : -ETIMEDOUT;
			}
			k_busy_wait(32);
		}
	}
	sys_write32(frame & ~LCDC_GENERATE, lcd + LCDC_FRAME_CTRL);
	sys_write32(transfer & ~LCDC_START, lcd + LCDC_TRANSFER);
	sys_write32(cfg->cs, cfg->gpio + GPIO_SET);
	sys_write32(mux, cfg->mux);
	sys_write32(cfg->clk | data_mask(cfg), cfg->gpio + GPIO_CLR);
	if (!lcdc_guard_ok(cfg)) {
		return -EFAULT;
	}
	return bus_idle(cfg) ? ret : -EIO;
}

int bes2700imp_lcdc_prepare(const struct bes2700imp_lcdc *cfg, uint16_t width, uint16_t height)
{
	const uintptr_t end = cfg->lcdc_buffer + cfg->lcdc_buffer_size;
	const uint32_t before = sys_read32(CMU_SYS_CLOCK) & CPU_SOURCE_MASK;
	int ret = configure_display_clocks();

	if (ret != 0) {
		return ret;
	}
	configure_frame_path(cfg->lcdc, width | ((uint32_t)height << 16));
	for (uintptr_t address = end - BES2700IMP_LCDC_GUARD_SIZE; address < end;
	     address += sizeof(uint16_t)) {
		sys_write16(LCDC_GUARD_WORD, address);
	}

	return ((sys_read32(CMU_SYS_CLOCK) & CPU_SOURCE_MASK) == before &&
		(sys_read32(CMU_OCLK_ENABLE) & DISPLAY_OCLKS) == DISPLAY_OCLKS &&
		bus_idle(cfg)) ? 0 : -EIO;
}

int bes2700imp_lcdc_write(const struct bes2700imp_lcdc *cfg,
			const struct display_buffer_descriptor *desc, const void *buffer)
{
	const uint8_t *pixels = buffer;
	uint8_t *staging = (uint8_t *)cfg->lcdc_buffer;
	size_t row_bytes = (size_t)desc->width * 2U;
	size_t pitch = ROUND_UP(row_bytes, 4U);
	size_t size = pitch * desc->height;

	if (size > cfg->lcdc_buffer_size - BES2700IMP_LCDC_GUARD_SIZE) {
		return -EINVAL;
	}
	if (!lcdc_guard_ok(cfg)) {
		return -EFAULT;
	}
	if (!bus_idle(cfg) ||
	    (sys_read32(cfg->lcdc + LCDC_FRAME_CTRL) & LCDC_GENERATE) != 0U ||
	    (sys_read32(cfg->lcdc + LCDC_TRANSFER) & LCDC_START) != 0U) {
		return -EBUSY;
	}
	if (pixels == staging) {
		if (desc->pitch * 2U != pitch) {
			return -EINVAL;
		}
	} else {
		for (unsigned int row = 0; row < desc->height; row++) {
			memcpy(staging + row * pitch, pixels + 2U * row * desc->pitch, row_bytes);
		}
	}
	if (IS_ENABLED(CONFIG_DCACHE)) {
		int ret = sys_cache_data_flush_range(staging, size);

		if (ret != 0) {
			return ret;
		}
	}
	barrier_dsync_fence_full();
	if (!lcdc_configure(cfg, desc->width, desc->height, pitch)) {
		return -EIO;
	}
	return lcdc_transfer(cfg);
}
