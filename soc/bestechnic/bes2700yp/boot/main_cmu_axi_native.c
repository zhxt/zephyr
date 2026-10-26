/* SPDX-License-Identifier: Apache-2.0 */

#include "main_cmu_axi_native.h"
#include "cmu_gate_native.h"
#include "cmu_timer_native.h"

__attribute__((section(".boot_bss.boot_main_cmu_axi_clk_map")))
uint8_t boot_main_cmu_axi_clk_map;
__attribute__((section(".boot_bss.boot_main_cmu_axi_rst_map")))
uint8_t boot_main_cmu_axi_rst_map;

__attribute__((section(".boot_text_flash.hal_cmu_axi_clock_enable"), noinline))
void hal_cmu_axi_clock_enable(uint32_t user)
{
	uint32_t key = boot_cmu_timer_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	uint8_t users = boot_main_cmu_axi_clk_map;

	if (users == 0U) {
		boot_cmu_gate_write(0x40080004U, 0x40U);
		(void)boot_cmu_gate_read(0x40080000U);
	}
	if (user < 3U) {
		boot_main_cmu_axi_clk_map = users | (uint8_t)(1U << user);
	}
	boot_cmu_timer_irq_unlock(key);
}

__attribute__((section(".boot_text_flash.hal_cmu_axi_reset_clear"), noinline))
void hal_cmu_axi_reset_clear(uint32_t user)
{
	uint32_t key = boot_cmu_timer_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	uint8_t users = boot_main_cmu_axi_rst_map;

	if (users == 0U) {
		boot_cmu_gate_write(0x400800ccU, 1U);
		(void)boot_cmu_gate_read(0x40080000U);
	}
	if (user < 3U) {
		boot_main_cmu_axi_rst_map = users | (uint8_t)(1U << user);
	}
	boot_cmu_timer_irq_unlock(key);
}
