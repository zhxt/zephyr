/* SPDX-License-Identifier: Apache-2.0 */

#include "cmu_frequency_native.h"
#include "cmu_gate_native.h"
#include "cmu_timer_native.h"

volatile uint8_t boot_cmu_force_sys_div
	__attribute__((section(".boot_bss.boot_cmu_force_sys_div")));

__attribute__((section(".boot_text_sram.hal_cmu_sys_get_freq"), noinline))
uint32_t hal_cmu_sys_get_freq(void)
{
	uint32_t value = boot_cmu_frequency_read(0x40000060U);

	if ((value & 0x800U) != 0U) {
		uint32_t divider;

		if ((value & 0x1000U) != 0U) {
			return 18U;
		}
		divider = boot_cmu_frequency_read(0x40000068U) & 3U;
		return divider == 0U ? 14U : divider == 1U ? 12U : 10U;
	}
	if ((value & 0x600U) == 0x400U) {
		return 14U;
	}
	if ((value & 0x600U) == 0x600U) {
		return 10U;
	}
	return (value & 0x100U) != 0U ? 6U : 0U;
}

__attribute__((section(".boot_text_flash.hal_cmu_sys_set_freq"), noinline))
int hal_cmu_sys_set_freq(uint32_t frequency)
{
	uint32_t select = 0xc000U;
	uint32_t divider = UINT32_MAX;
	uint32_t enable;
	uint32_t disable;
	uint32_t force;
	uint32_t key;
	uint32_t value;

	if (frequency > 18U) {
		return 1;
	}
	if (frequency >= 15U) {
		select = (frequency - 15U) << 14;
		enable = 0x1b00U;
		disable = 0x20U;
	} else if (frequency >= 11U) {
		select = (frequency - 11U) << 14;
		divider = 0U;
		enable = 0xb20U;
		disable = 0x1000U;
	} else if (frequency >= 7U) {
		select = (frequency - 7U) << 14;
		enable = 0x700U;
		disable = 0x1820U;
	} else {
		enable = frequency == 0U ? 0x200U : 0x300U;
		disable = frequency == 0U ? 0x1de0U : 0x1c60U;
	}
	force = boot_cmu_force_sys_div;
	if (force != 0U) {
		enable &= ~0x1020U;
		disable &= ~0x1020U;
		if (force == 1U) {
			enable |= 0x1000U;
			disable |= 0x20U;
		} else {
			divider = force - 2U;
			enable |= 0x20U;
			disable |= 0x1000U;
		}
	}
	key = boot_cmu_timer_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	value = boot_cmu_gate_read(0x40000068U);
	boot_cmu_gate_write(0x40000068U, (value & ~0xc000U) | select);
	if (divider != UINT32_MAX) {
		/* Preserve the second read if an intervening update changed this register. */
		value = boot_cmu_gate_read(0x40000068U);
		boot_cmu_gate_write(0x40000068U, (value & ~3U) | (divider & 3U));
	}
	boot_cmu_timer_irq_unlock(key);
	if ((enable & 0x800U) != 0U) {
		boot_cmu_gate_write(0x40000060U, 0x20U);
		boot_cmu_gate_write((enable & 0x1000U) != 0U ? 0x40000060U : 0x40000064U,
				    0x1000U);
		boot_cmu_gate_write(0x40000060U, enable);
		boot_cmu_gate_write(0x40000064U, disable);
	} else {
		boot_cmu_gate_write(0x40000060U, enable);
		boot_cmu_gate_write(0x40000064U, disable & ~0x1020U);
		boot_cmu_gate_write(0x40000064U, 0x1000U);
		boot_cmu_gate_write(0x40000064U, 0x20U);
	}
	return 0;
}
