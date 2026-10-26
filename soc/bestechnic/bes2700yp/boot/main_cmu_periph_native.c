/* SPDX-License-Identifier: Apache-2.0 */

#include "main_cmu_periph_native.h"
#include "cmu_gate_native.h"
#include "cmu_timer_native.h"

__attribute__((section(".text.hal_cmu_spi_set_freq"), noinline))
int hal_cmu_spi_set_freq(uint32_t selector)
{
	uint32_t key = boot_cmu_timer_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	int result = 1;

	if (selector < 2U) {
		uint32_t value = boot_cmu_gate_read(0x500000a8U);

		boot_cmu_gate_write(0x500000a8U, (value & ~0xe0000U) | (selector << 17));
		result = 0;
	}
	boot_cmu_timer_irq_unlock(key);
	return result;
}

__attribute__((section(".text.hal_cmu_slcd_set_freq"), noinline))
int hal_cmu_slcd_set_freq(uint32_t selector)
{
	uint32_t key = boot_cmu_timer_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	int result = 1;

	if (selector < 2U) {
		uint32_t value = boot_cmu_gate_read(0x500000a8U);

		boot_cmu_gate_write(0x500000a8U, (value & ~0x1c00000U) | (selector << 22));
		result = 0;
	}
	boot_cmu_timer_irq_unlock(key);
	return result;
}
