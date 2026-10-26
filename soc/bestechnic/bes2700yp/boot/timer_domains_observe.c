/* SPDX-License-Identifier: Apache-2.0 */

#include "timer_domains_observe.h"
#ifdef BOOT_TIMER_DOMAINS_HOST
#include "tests/timer_domains_host.h"
#else
#include "arch.h"
#include "boot_contract.h"
#define BOOT_TD_READ(address) (*(__IM uint32_t *)(uintptr_t)(address))
#endif

/* Local fixed-binary integration observations, not public BES documentation.
 * Only addresses already read by the retained timer and adapter are observed.
 * This capture runs after the retained early clock preparation and normal CRT.
 */
#define BOOT_TD_SLOW 0x40080050U
#define BOOT_TD_FAST 0x40002004U
#define BOOT_TD_CONTROL 0x40002008U
#ifndef __section
#define __section(name) __attribute__((__section__(name)))
#endif

__section(".boot_text_sram.boot_timer_domains_capture")
__attribute__((noinline))
void boot_timer_domains_capture(struct boot_timer_domains_point *points,
				enum boot_timer_phase phase)
{
	struct boot_timer_domains_point *point;

	if ((uint32_t)phase >= BOOT_TIMER_PHASE_COUNT) {
		return;
	}
	point = &points[phase];

	point->primask_before = __get_PRIMASK();
	point->basepri_before = __get_BASEPRI();
	point->control_before = BOOT_TD_READ(BOOT_TD_CONTROL);
	point->slow_before = BOOT_TD_READ(BOOT_TD_SLOW);
	point->fast_before = BOOT_TD_READ(BOOT_TD_FAST);
	/* A bounded instruction gap, independent of either observed counter.
	 * Zero slow-counter progress in this short window is permitted.
	 */
	for (uint32_t i = 0U; i < 512U; i++) {
		__NOP();
	}
	point->slow_after = BOOT_TD_READ(BOOT_TD_SLOW);
	point->fast_after = BOOT_TD_READ(BOOT_TD_FAST);
	point->control_after = BOOT_TD_READ(BOOT_TD_CONTROL);
	point->primask_after = __get_PRIMASK();
	point->basepri_after = __get_BASEPRI();
}

void boot_timer_domains_dump(const struct boot_timer_domains_point *points)
{
	static const char *const phases[BOOT_TIMER_PHASE_COUNT] = {
		"before_pmu", "before_clock", "after_clock",
		"after_timer_open", "before_handoff",
	};

	/* Print only after the caller has collected all five samples. */
	for (uint32_t i = 0U; i < BOOT_TIMER_PHASE_COUNT; i++) {
		bth_puts("zephyr_bth timer_domains version=1 phase=");
		bth_puts(phases[i]);
		bth_field(" ctrl0=", points[i].control_before);
		bth_field(" slow0=", points[i].slow_before);
		bth_field(" fast0=", points[i].fast_before);
		bth_field(" slow1=", points[i].slow_after);
		bth_field(" fast1=", points[i].fast_after);
		bth_field(" ctrl1=", points[i].control_after);
		bth_field(" primask0=", points[i].primask_before);
		bth_field(" basepri0=", points[i].basepri_before);
		bth_field(" primask1=", points[i].primask_after);
		bth_field(" basepri1=", points[i].basepri_after);
		bth_end();
	}
	bth_puts("zephyr_bth timer_domains complete version=1 samples=5 !\r\n");
}
