/* SPDX-License-Identifier: Apache-2.0 */

#include "timer_native.h"
#include "timer_wait.h"

#ifndef __section
#define __section(name) __attribute__((__section__(name)))
#endif

/* Fixed ABI conversion factors, not measurements of physical frequency. */
#define FAST_TICKS_PER_US 6U
#define SLOW_TICKS_PER_MS 16U
#define WAIT_CHUNK_MAX 0x7fffffffU

__section(".boot_text_sram.hal_sys_timer_get")
uint32_t hal_sys_timer_get(void)
{
	return boot_timer_slow_read32();
}

uint32_t hal_sys_timer_get_in_sleep(void)
	__attribute__((alias("hal_sys_timer_get")));

__typeof__(hal_sys_timer_get) boot_timer_read_ticks32
	__attribute__((alias("hal_sys_timer_get")));

__section(".boot_text_sram.hal_sys_timer_delay")
void hal_sys_timer_delay(uint32_t ticks)
{
	boot_timer_wait_up32(ticks);
}

void hal_sys_timer_delay_in_sleep(uint32_t ticks)
	__attribute__((alias("hal_sys_timer_delay")));

__section(".boot_text_sram.boot_timer_wait_fast32")
static void wait_fast(uint32_t ticks)
{
	uint32_t start = boot_timer_fast_read32();

	for (;;) {
		uint32_t current = boot_timer_fast_read32();

		if ((uint32_t)(start - current) >= ticks) {
			break;
		}
	}
}

__section(".boot_text_sram.hal_sys_timer_delay_us")
void hal_sys_timer_delay_us(uint32_t us)
{
	uint64_t remaining = (uint64_t)us * FAST_TICKS_PER_US;

	do {
		uint32_t chunk = remaining > WAIT_CHUNK_MAX ?
				 WAIT_CHUNK_MAX : (uint32_t)remaining;

		wait_fast(chunk);
		remaining -= chunk;
	} while (remaining != 0U);
}

__section(".boot_text_sram.osDelay")
int osDelay(uint32_t ms)
{
	uint64_t remaining = (uint64_t)ms * SLOW_TICKS_PER_MS;

	do {
		uint32_t chunk = remaining > WAIT_CHUNK_MAX ?
				 WAIT_CHUNK_MAX : (uint32_t)remaining;

		boot_timer_wait_up32(chunk);
		remaining -= chunk;
	} while (remaining != 0U);
	return 0;
}

__section(".boot_text_flash.hal_sys_timer_open")
void hal_sys_timer_open(void)
{
	boot_timer_hw_open();
}
