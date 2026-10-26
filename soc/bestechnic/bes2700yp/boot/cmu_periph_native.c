/* SPDX-License-Identifier: Apache-2.0 */

#include "cmu_periph_native.h"
#include "cmu_timer_native.h"

__attribute__((section(".boot_text_flash.hal_cmu_uart0_set_freq"), noinline))
int hal_cmu_uart0_set_freq(uint32_t selector)
{
	uint32_t key = boot_cmu_timer_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	int result = 1;

	if (selector < 2U) {
		boot_cmu_uart_modify(0x1cU, selector * 4U);
		result = 0;
	}
	boot_cmu_timer_irq_unlock(key);
	return result;
}

__attribute__((section(".boot_text_flash.hal_cmu_ispi_set_freq"), noinline))
int hal_cmu_ispi_set_freq(uint32_t selector)
{
	uint32_t key = boot_cmu_timer_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	int result = 1;

	if (selector < 2U) {
		boot_cmu_timer_modify(0x800000U, selector != 0U ? 0x800000U : 0U);
		result = 0;
	}
	boot_cmu_timer_irq_unlock(key);
	return result;
}
