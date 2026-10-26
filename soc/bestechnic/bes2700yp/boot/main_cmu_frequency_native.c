/* SPDX-License-Identifier: Apache-2.0 */

#include "main_cmu_frequency_native.h"
#include "cmu_pll_native.h"

/* These selectors are fixed-build observations, without an asserted MHz mapping. */
int hal_cmu_mem_set_freq(uint32_t frequency)
{
	const uintptr_t base = 0x50000000U;
	uint32_t gated;
	uint32_t previous;
	uint32_t select;
	uint32_t clear;

	if (frequency > 18U) {
		return 1;
	}
	gated = boot_main_cmu_frequency_read(base + 0x14U) & 0x200000U;
	if (gated != 0U) {
		boot_main_cmu_frequency_write(base + 0x14U, 0x200000U);
		boot_main_cmu_frequency_write(base + 4U, 0x40000U);
		(void)boot_main_cmu_frequency_read(base + 4U);
		boot_main_cmu_frequency_barrier();
	}
	previous = boot_main_cmu_frequency_read(base + 0x60U);
	if (frequency <= 10U) {
		select = frequency <= 6U ? 0U : 0x200000U;
		clear = frequency <= 6U ? 0x780000U : 0x580000U;
	} else {
		select = 0x400000U;
		clear = 0x80000U;
		if ((previous & 0x400000U) == 0U) {
			uint32_t key = boot_main_cmu_frequency_irq_lock(
				(uint32_t)(uintptr_t)__builtin_return_address(0));
			uint32_t value = boot_main_cmu_frequency_read(base + 0xa8U);

			boot_main_cmu_frequency_write(base + 0xa8U, value & ~0x20000000U);
			boot_main_cmu_frequency_irq_unlock(key);
			(void)hal_cmu_pll_enable(0U, 10U);
			boot_main_cmu_frequency_barrier();
		}
	}
	boot_main_cmu_frequency_write(base + 0x60U, select);
	boot_main_cmu_frequency_write(base + 0x64U, clear);
	if (gated != 0U) {
		boot_main_cmu_frequency_write(base, 0x40000U);
		boot_main_cmu_frequency_write(base + 0x10U, 0x200000U);
	}
	(void)boot_main_cmu_frequency_read(base + 0x60U);
	boot_main_cmu_frequency_barrier();
	if (frequency <= 10U && (previous & 0x400000U) != 0U) {
		(void)hal_cmu_pll_disable(0U, 10U);
		boot_main_cmu_frequency_barrier();
	}
	return 0;
}

/* Inline the shared protocol so the BTH body has no Flash-resident helper. */
static inline __attribute__((always_inline))
int set_flash(uint32_t frequency, uintptr_t base, uint32_t gate,
	      uintptr_t divider, uint32_t divider_mask, uint32_t divider_one,
	      uint32_t caller)
{
	uint32_t gated;
	uint32_t select;
	uint32_t clear;

	if (frequency > 18U) {
		return 1;
	}
	gated = boot_main_cmu_frequency_read(base + 0x14U) & 2U;
	if (gated != 0U) {
		boot_main_cmu_frequency_write(base + 0x14U, 2U);
		boot_main_cmu_frequency_write(base + 4U, gate);
		(void)boot_main_cmu_frequency_read(base + 4U);
		boot_main_cmu_frequency_barrier();
	}
	if (frequency <= 6U) {
		select = 0U;
		clear = 0x1fU;
	} else if (frequency <= 10U) {
		select = 4U;
		clear = 0x1bU;
	} else if (frequency <= 14U) {
		uint32_t key = boot_main_cmu_frequency_irq_lock(caller);
		uint32_t value = boot_main_cmu_frequency_read(divider);

		boot_main_cmu_frequency_write(divider, (value & ~divider_mask) |
					     (frequency <= 12U ? divider_one : 0U));
		boot_main_cmu_frequency_irq_unlock(key);
		select = 9U;
		clear = 0x10U;
	} else {
		select = 0x18U;
		clear = 1U;
	}
	boot_main_cmu_frequency_write(base + 0x60U, select);
	boot_main_cmu_frequency_write(base + 0x64U, clear);
	if (gated != 0U) {
		boot_main_cmu_frequency_write(base, gate);
		boot_main_cmu_frequency_write(base + 0x10U, 2U);
	}
	(void)boot_main_cmu_frequency_read(base + 0x60U);
	boot_main_cmu_frequency_barrier();
	return 0;
}

int hal_cmu_sys_flash_set_freq(uint32_t frequency)
{
	return set_flash(frequency, 0x50000000U, 0x40U, 0x500000a8U, 0x60U, 0x20U,
			 (uint32_t)(uintptr_t)__builtin_return_address(0));
}

__attribute__((section(".boot_text_sram.hal_cmu_bth_flash_set_freq"), noinline))
int hal_cmu_bth_flash_set_freq(uint32_t frequency)
{
	return set_flash(frequency, 0x40000000U, 8U, 0x40000068U, 0xcU, 4U,
			 (uint32_t)(uintptr_t)__builtin_return_address(0));
}

int hal_cmu_flash1_set_freq(uint32_t frequency)
	__attribute__((alias("hal_cmu_sys_flash_set_freq")));
int hal_cmu_flash_set_freq(uint32_t frequency)
	__attribute__((alias("hal_cmu_bth_flash_set_freq")));
