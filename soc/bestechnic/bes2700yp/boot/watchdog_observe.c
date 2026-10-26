/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */

#include "watchdog_observe.h"
#ifdef BES_BOOT_WDT_CMSDK_STOP
#include "watchdog_stop.h"
#endif
#ifdef BOOT_WDT_OBSERVE_HOST
#include "tests/watchdog_observe_host.h"
#else
#include "arch.h"
#include "boot_contract.h"
#define BOOT_WDT_READ(address) (*(__IM uint32_t *)(uintptr_t)(address))
#endif

/* Fixed local binary integration contract, not a public BES register map.
 * Observe only offsets already accessed by the retained hal_wdt_stop(0).
 * Clock/power accessibility still relies on the original startup and PMU path.
 */
#define BOOT_WDT_CONTROL 0x40082008U
#define BOOT_WDT_LOCK 0x40082c00U

struct boot_wdt_observation {
	uint32_t control;
	uint32_t lock;
	uint32_t primask_before;
	uint32_t basepri_before;
	uint32_t primask_after;
	uint32_t basepri_after;
};

static struct boot_wdt_observation points[BOOT_WDT_PHASE_COUNT];
static uint32_t sample_count;
static uint32_t phase_mask;
static uint32_t stop_calls;
static uint32_t stop_id;
static int stop_result;

__attribute__((noinline))
void boot_wdt_observe_capture(enum boot_wdt_phase phase)
{
	struct boot_wdt_observation *point = &points[phase];

	point->primask_before = __get_PRIMASK();
	point->basepri_before = __get_BASEPRI();
	point->control = BOOT_WDT_READ(BOOT_WDT_CONTROL);
	point->lock = BOOT_WDT_READ(BOOT_WDT_LOCK);
	point->primask_after = __get_PRIMASK();
	point->basepri_after = __get_BASEPRI();
	sample_count++;
	phase_mask |= 1U << phase;
}

__attribute__((noinline))
int __wrap_hal_wdt_stop(uint32_t id)
{
	stop_id = id;
	stop_calls++;
	boot_wdt_observe_capture(BOOT_WDT_BEFORE_STOP);
	stop_result = __real_hal_wdt_stop(id);
	boot_wdt_observe_capture(BOOT_WDT_AFTER_STOP);
	return stop_result;
}

void boot_wdt_observe_dump(void)
{
	static const char *const phases[BOOT_WDT_PHASE_COUNT] = {
		"before_stop", "after_stop", "before_handoff",
	};

	/* No UART output occurs inside either stop capture window. */
	for (uint32_t i = 0U; i < BOOT_WDT_PHASE_COUNT; i++) {
		bth_puts("zephyr_bth watchdog_probe version=1 phase=");
		bth_puts(phases[i]);
		bth_field(" ctrl=", points[i].control);
		bth_field(" lock=", points[i].lock);
		bth_field(" primask0=", points[i].primask_before);
		bth_field(" basepri0=", points[i].basepri_before);
		bth_field(" primask1=", points[i].primask_after);
		bth_field(" basepri1=", points[i].basepri_after);
		bth_end();
	}
	bth_puts("zephyr_bth watchdog_probe complete version=1 ");
#ifdef BES_BOOT_WDT_CMSDK_STOP
	bth_puts("provider=cmsdk-stop samples=");
#else
	bth_puts("provider=vendor-stop samples=");
#endif
	bth_dec(sample_count);
	bth_puts(" calls=");
	bth_dec(stop_calls);
	bth_field(" id=", stop_id);
	bth_field(" rc=", (uint32_t)stop_result);
	bth_field(" phase_mask=", phase_mask);
	bth_end();
#ifdef BES_BOOT_WDT_CMSDK_STOP
	boot_wdt_stop_dump();
#endif
}
