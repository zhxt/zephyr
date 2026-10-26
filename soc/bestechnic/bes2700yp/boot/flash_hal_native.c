/* SPDX-License-Identifier: Apache-2.0 */

#include "flash_hal_native.h"
#include "flaship_native.h"
#include "main_cmu_frequency_native.h"
#include "nor_native.h"

#define SRAM(name) __attribute__((section(".boot_text_sram." #name), noinline))

void hal_sys_timer_delay(uint32_t ticks);

SRAM(hal_norflash_set_boot_freq)
void hal_norflash_set_boot_freq(uint32_t frequency)
{
	hal_sys_timer_delay(4U);
	if (norflaship_is_busy(0U) != 0U) {
		hal_sys_timer_delay(4U);
	}
	norflaship_div(0U, 8U);
	(void)norflash_init_sample_delay_by_div(0U, 8U);
	(void)hal_cmu_flash_set_freq(frequency);
}
