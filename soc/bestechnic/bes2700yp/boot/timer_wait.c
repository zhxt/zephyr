/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */

#include "timer_wait.h"

#ifndef __section
#define __section(name) __attribute__((__section__(name)))
#endif

/* The unsigned elapsed comparison follows Zephyr kernel/busy_wait.c.
 * A separate backend supplies raw ticks; no kernel or device API is used.
 */
__section(".boot_text_sram.boot_timer_wait_up32")
void boot_timer_wait_up32(uint32_t ticks)
{
	uint32_t start = boot_timer_read_ticks32();

	for (;;) {
		uint32_t current = boot_timer_read_ticks32();

		if ((uint32_t)(current - start) >= ticks) {
			break;
		}
	}
}
