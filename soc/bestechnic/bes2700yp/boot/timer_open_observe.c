/* SPDX-License-Identifier: Apache-2.0 */

#include "timer_open_observe.h"
#include <stddef.h>
#ifdef BOOT_TIMER_OPEN_HOST
#include "tests/timer_open_host.h"
#else
#include "arch.h"
#include "boot_contract.h"
#define BOOT_TIMER_OPEN_READ(address) (*(__IM uint32_t *)(uintptr_t)(address))
#endif

#ifndef __section
#define __section(name) __attribute__((__section__(name)))
#endif

/* The early CRT zero pass initializes this input section before hal_cmu_setup.
 * It remains disjoint from every later copy and zero range. No normal BSS or
 * initialized data is used by the wrapper or its capture window.
 */
struct boot_timer_open_state boot_timer_open_trace
	__section(".boot_bss.timer_open_observe");

_Static_assert(sizeof(struct boot_timer_open_state) == 88U,
	       "bounded early timer observation storage");

/* Fixed local integration addresses, not public BES register documentation.
 * Read only after the real open returns, never before clock preparation.
 */
__section(".boot_text_sram.boot_timer_open_capture")
__attribute__((noinline))
void boot_timer_open_capture(struct boot_timer_open_point *point)
{
	point->primask_before = __get_PRIMASK();
	point->basepri_before = __get_BASEPRI();
	point->control_before = BOOT_TIMER_OPEN_READ(0x40002008U);
	point->fast_before = BOOT_TIMER_OPEN_READ(0x40002004U);
	for (uint32_t i = 0U; i < 512U; i++) {
		__NOP();
	}
	point->fast_after = BOOT_TIMER_OPEN_READ(0x40002004U);
	point->control_after = BOOT_TIMER_OPEN_READ(0x40002008U);
	point->primask_after = __get_PRIMASK();
	point->basepri_after = __get_BASEPRI();
}

__section(".boot_text_flash.boot_timer_open_wrap")
__attribute__((noinline))
void __wrap_hal_sys_timer_open(void)
{
	uint32_t slot = boot_timer_open_trace.calls;
	struct boot_timer_open_point *point = NULL;

	if (slot < BOOT_TIMER_OPEN_CAPACITY) {
		boot_timer_open_trace.calls = slot + 1U;
		point = &boot_timer_open_trace.points[slot];
		/* Only architectural reads before hardware initialization. */
		point->entry_primask = __get_PRIMASK();
		point->entry_basepri = __get_BASEPRI();
	} else {
		boot_timer_open_trace.overflow = 1U;
	}
	/* Run the original initializer even when the record is full. */
	__real_hal_sys_timer_open();
	if (point != NULL) {
		boot_timer_open_capture(point);
	}
}

void boot_timer_open_dump(void)
{
	for (uint32_t i = 0U; i < BOOT_TIMER_OPEN_CAPACITY; i++) {
		const struct boot_timer_open_point *point =
			&boot_timer_open_trace.points[i];

		bth_puts("zephyr_bth timer_open version=1 phase=");
		bth_puts(i == 0U ? "initial" : "adapter");
		bth_field(" entry_primask=", point->entry_primask);
		bth_field(" entry_basepri=", point->entry_basepri);
		bth_field(" primask0=", point->primask_before);
		bth_field(" basepri0=", point->basepri_before);
		bth_field(" ctrl0=", point->control_before);
		bth_field(" fast0=", point->fast_before);
		bth_field(" fast1=", point->fast_after);
		bth_field(" ctrl1=", point->control_after);
		bth_field(" primask1=", point->primask_after);
		bth_field(" basepri1=", point->basepri_after);
		bth_end();
	}
	bth_puts("zephyr_bth timer_open complete version=1");
	bth_field(" calls=", boot_timer_open_trace.calls);
	bth_field(" overflow=", boot_timer_open_trace.overflow);
	bth_end();
}
