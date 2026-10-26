/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */

#ifndef BES_BOOT_WATCHDOG_OBSERVE_H_
#define BES_BOOT_WATCHDOG_OBSERVE_H_

#include <stdint.h>

enum boot_wdt_phase {
	BOOT_WDT_BEFORE_STOP,
	BOOT_WDT_AFTER_STOP,
	BOOT_WDT_BEFORE_HANDOFF,
	BOOT_WDT_PHASE_COUNT,
};

void boot_wdt_observe_capture(enum boot_wdt_phase phase);
void boot_wdt_observe_dump(void);
int __wrap_hal_wdt_stop(uint32_t id);
int __real_hal_wdt_stop(uint32_t id);

#ifdef BES_BOOT_WATCHDOG_OBSERVE
#define BOOT_WDT_CAPTURE(phase) boot_wdt_observe_capture(phase)
#define BOOT_WDT_DUMP() boot_wdt_observe_dump()
#else
#define BOOT_WDT_CAPTURE(phase) ((void)0)
#define BOOT_WDT_DUMP() ((void)0)
#endif

#endif /* BES_BOOT_WATCHDOG_OBSERVE_H_ */
