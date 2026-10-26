/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "arch.h"
#include "boot_contract.h"
#include "timer_observe.h"

/* Sample the two registers already read by the imported adapter/diagnostic
 * contract. No Load, Control, CMU, IRQ or debug-register writes are added.
 * These samples rely on the retained BootInit having prepared the peripheral.
 */
#ifdef BES_BOOT_TIMER_OBSERVE_SRAM
__attribute__((section(".boot_text_sram.boot_timer_observe_capture"), noinline))
#endif
void boot_timer_observe_capture(struct boot_timer_observation *points, enum boot_timer_phase phase)
{
	struct boot_timer_observation *point = &points[phase];

	point->control_before = BTH_REG(BTH_TIMER_BASE + 8U);
	point->count_before = BTH_REG(BTH_TIMER_BASE + 4U);
	/* An instruction gap checks progress without using a timer-based delay.
	 * Its duration depends on CPU clock and execution conditions.
	 */
	for (uint32_t i = 0U; i < 512U; i++) {
		__NOP();
	}
	point->count_after = BTH_REG(BTH_TIMER_BASE + 4U);
	point->control_after = BTH_REG(BTH_TIMER_BASE + 8U);
}

void boot_timer_observe_dump(const struct boot_timer_observation *points)
{
#ifdef BES_BOOT_SKIP_ADAPTER_TIMER_OPEN
	static const char probe_prefix[] = "zephyr_bth timer_probe version=2 phase=";
	static const char timer_phase[] = "after_timer_check";
#else
	static const char probe_prefix[] = "zephyr_bth timer_probe version=1 phase=";
	static const char timer_phase[] = "after_timer_open";
#endif
	static const char *const phases[BOOT_TIMER_PHASE_COUNT] = {
		"before_pmu", "before_clock", "after_clock", timer_phase, "before_handoff",
	};

	/* Defer UART output until every snapshot is collected. */
#ifdef BES_BOOT_SKIP_ADAPTER_TIMER_OPEN
	bth_puts("zephyr_bth timer_reopen=skipped");
	bth_end();
#endif
	for (uint32_t i = 0U; i < BOOT_TIMER_PHASE_COUNT; i++) {
		bth_puts(probe_prefix);
		bth_puts(phases[i]);
		bth_field(" ctrl0=", points[i].control_before);
		bth_field(" count0=", points[i].count_before);
		bth_field(" count1=", points[i].count_after);
		bth_field(" ctrl1=", points[i].control_after);
		bth_end();
	}
	bth_puts("zephyr_bth timer_probe complete samples=");
	bth_dec(BOOT_TIMER_PHASE_COUNT);
	bth_end();
}
