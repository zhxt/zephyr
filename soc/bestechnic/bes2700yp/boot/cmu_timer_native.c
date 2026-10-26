/* SPDX-License-Identifier: Apache-2.0 */

#include "cmu_timer_native.h"

__attribute__((section(".boot_text_flash.hal_cmu_timer1_select_fast"), noinline))
void hal_cmu_timer1_select_fast(void)
{
	uint32_t key = boot_cmu_timer_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));

	boot_cmu_timer_modify(0U, 0x200U);
	boot_cmu_timer_irq_unlock(key);
}

__attribute__((section(".boot_text_flash.hal_cmu_timer2_select_fast"), noinline))
void hal_cmu_timer2_select_fast(void)
{
	uint32_t key = boot_cmu_timer_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));

	boot_cmu_timer_modify(0U, 0x400U);
	boot_cmu_timer_irq_unlock(key);
}

__attribute__((section(".boot_text_flash.hal_cmu_fast_timer_init"), noinline))
int hal_cmu_fast_timer_init(void)
{
	boot_cmu_timer_modify(0x60000000U, 0U);
	return 0;
}
