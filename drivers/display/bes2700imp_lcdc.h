/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DISPLAY_BES2700IMP_LCDC_H_
#define ZEPHYR_DRIVERS_DISPLAY_BES2700IMP_LCDC_H_

#include <stdint.h>
#include <zephyr/drivers/display.h>

#define BES2700IMP_LCDC_GUARD_SIZE 64U

struct bes2700imp_lcdc {
	uintptr_t gpio;
	uintptr_t mux;
	uintptr_t reset_gpio;
	uintptr_t reset_mux;
	uintptr_t lcdc;
	uintptr_t lcdc_buffer;
	size_t lcdc_buffer_size;
	uint32_t clk;
	uint32_t cs;
	uint32_t reset;
	uint8_t d0;
};

int bes2700imp_lcdc_prepare(const struct bes2700imp_lcdc *bus, uint16_t width, uint16_t height);
int bes2700imp_lcdc_command(const struct bes2700imp_lcdc *bus, uint8_t cmd, const uint8_t *data,
			  size_t size);
int bes2700imp_lcdc_read(const struct bes2700imp_lcdc *bus, uint8_t cmd, uint8_t *data);
/* Returns -EFAULT if the DMA buffer guard is corrupted. */
int bes2700imp_lcdc_write(const struct bes2700imp_lcdc *bus,
			const struct display_buffer_descriptor *desc, const void *buffer);

#endif
