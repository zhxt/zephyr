/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include "analog_native.h"
#include "cmu_timer_native.h"
#include "main_cmu_leaf_native.h"

#define NORMAL(name) __attribute__((section(".text." #name), noinline))
#define BOOT(name) __attribute__((section(".boot_text_flash." #name), noinline))
#define EARLY(name) __attribute__((section(".boot_bss." #name)))

EARLY(boot_analog_cs) uint8_t boot_analog_cs;
EARLY(boot_analog_revision) uint8_t boot_analog_revision;
EARLY(boot_analog_control) struct boot_spi_control boot_analog_control;
EARLY(boot_analog_internal_control) struct boot_spi_control boot_analog_internal_control;

static const struct boot_spi_config default_config = {
	.rate = 6000000U,
	.flags = 0x20U,
	.tx_bits = 27U,
	.rx_bits = 27U,
};

__attribute__((always_inline)) static inline void select_cs(uint32_t cs)
{
	uint32_t previous = boot_analog_cs;

	if (previous != cs) {
		if (boot_analog_revision <= 1U) {
			if (cs == 8U) {
				hal_ispi_enable_ctrl(&boot_analog_internal_control);
			} else if (previous == 8U) {
				hal_ispi_enable_ctrl(&boot_analog_control);
			}
		}
		hal_ispi_activate_cs(cs);
		boot_analog_cs = (uint8_t)cs;
	}
}

NORMAL(hal_analogif_reg_read)
int hal_analogif_reg_read(uint32_t address, uint16_t *value)
{
	uint32_t page = (address >> 8) & 15U;

	if (page > 4U) {
		return -1;
	}
	uint32_t caller = (uint32_t)(uintptr_t)__builtin_return_address(0);
	uint32_t key = boot_cmu_timer_irq_lock(caller);
	uint32_t received = 0U;
	uint32_t frame;

	select_cs(address >> 12);
	if (page != 0U) {
		frame = (0xa000U | (page << 4)) << 2;
		(void)hal_ispi_send(&frame, sizeof(frame));
	}
	frame = 0x04000000U | ((address & 255U) << 18);
	int result = hal_ispi_recv(&frame, &received, sizeof(frame));

	if (result == 0) {
		*value = (uint16_t)(received >> 2);
	}
	if (page != 0U) {
		frame = 0x00028000U;
		(void)hal_ispi_send(&frame, sizeof(frame));
	}
	boot_cmu_timer_irq_unlock(key);
	return result;
}

NORMAL(hal_analogif_reg_write)
int hal_analogif_reg_write(uint32_t address, uint32_t value)
{
	uint32_t page = (address >> 8) & 15U;

	if (page > 4U) {
		return -1;
	}
	uint32_t caller = (uint32_t)(uintptr_t)__builtin_return_address(0);
	uint32_t key = boot_cmu_timer_irq_lock(caller);
	uint32_t frame;

	select_cs(address >> 12);
	if (page != 0U) {
		frame = (0xa000U | (page << 4)) << 2;
		(void)hal_ispi_send(&frame, sizeof(frame));
	}
	frame = (value | ((address & 255U) << 16)) << 2;
	int result = hal_ispi_send(&frame, sizeof(frame));

	if (page != 0U) {
		frame = 0x00028000U;
		(void)hal_ispi_send(&frame, sizeof(frame));
	}
	boot_cmu_timer_irq_unlock(key);
	return result;
}

BOOT(hal_analogif_open)
int hal_analogif_open(void)
{
	struct boot_spi_config config;
	uint16_t identity = 0U;

	boot_analog_revision = (uint8_t)hal_cmu_get_aon_revision_id();
	(void)memcpy(&config, &default_config, sizeof(config));
	if (boot_analog_revision <= 1U) {
		(void)hal_spi_init_ctrl(&config, &boot_analog_control);
		config.rate = 1000000U;
		(void)hal_spi_init_ctrl(&config, &boot_analog_internal_control);
	}
	int result = hal_ispi_open(&config);

	if (result != 0) {
		return result;
	}
	boot_analog_cs = 8U;
	hal_ispi_activate_cs(8U);
	result = hal_analogif_reg_read(0x8000U, &identity);
	if (result != 0) {
		return result;
	}
	return (identity >> 4) == 0x160U ? 0 : -1;
}
