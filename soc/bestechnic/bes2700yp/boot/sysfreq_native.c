/* SPDX-License-Identifier: Apache-2.0 */

#include "sysfreq_native.h"

/* Normal CRT must neither clear these fields nor restore their initial values. */
__attribute__((section(".boot_data.sysfreq_top")))
volatile uint8_t boot_sysfreq_top_user = 42U;
__attribute__((section(".boot_data.sysfreq_minimum")))
volatile uint8_t boot_sysfreq_minimum;
__attribute__((section(".boot_bss.sysfreq_requests")))
_Alignas(4) volatile uint8_t boot_sysfreq_requests[44];

_Static_assert(sizeof(boot_sysfreq_requests) == 44U, "fixed early request storage");

__attribute__((always_inline))
static inline uint8_t effective_frequency(uint32_t requested, uint8_t minimum)
{
	uint32_t value = requested == 0U ? 6U : requested;

	return (uint8_t)(value < minimum ? minimum : value);
}

__attribute__((section(".boot_text_flash.hal_sysfreq_req"), noinline))
int hal_sysfreq_req(uint32_t user, uint32_t frequency)
{
	uint32_t key;
	uint32_t previous;
	uint8_t top;
	uint8_t selected;
	uint8_t minimum;
	uint8_t before;
	uint8_t after;
	int increasing;

	if (user >= 42U) {
		return 1;
	}
	if (frequency > 18U) {
		return 2;
	}
	key = boot_sysfreq_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	top = boot_sysfreq_top_user;
	previous = top < 42U ? boot_sysfreq_requests[top] : hal_cmu_sys_get_freq();
	boot_sysfreq_requests[user] = (uint8_t)frequency;
	if (frequency == previous) {
		boot_sysfreq_top_user = (uint8_t)user;
		goto restore;
	}
	increasing = frequency > previous;
	selected = (uint8_t)frequency;
	if (increasing) {
		boot_sysfreq_top_user = (uint8_t)user;
	} else if (boot_sysfreq_top_user == user) {
		uint8_t winner = 0U;

		selected = boot_sysfreq_requests[0];
		for (uint32_t index = 1U; index < 42U; index++) {
			uint8_t request = boot_sysfreq_requests[index];

			if (request > selected) {
				selected = request;
				winner = (uint8_t)index;
			}
		}
		boot_sysfreq_top_user = winner;
		if (selected == previous) {
			goto restore;
		}
	} else if (boot_sysfreq_top_user == 42U) {
		boot_sysfreq_top_user = (uint8_t)user;
	} else {
		goto restore;
	}
	minimum = boot_sysfreq_minimum;
	before = effective_frequency(previous, minimum);
	after = effective_frequency(selected, minimum);
	if (before != after) {
		if (increasing) {
			pmu_sys_freq_config(after);
			(void)hal_cmu_sys_set_freq(after);
		} else {
			(void)hal_cmu_sys_set_freq(after);
			pmu_sys_freq_config(after);
		}
	}
restore:
	boot_sysfreq_irq_unlock(key);
	return 0;
}
