/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BES_BOOT_TIMER_OBSERVE_H
#define BES_BOOT_TIMER_OBSERVE_H

#include <stdint.h>

enum boot_timer_phase {
	BOOT_TIMER_BEFORE_PMU,
	BOOT_TIMER_BEFORE_CLOCK,
	BOOT_TIMER_AFTER_CLOCK,
	BOOT_TIMER_AFTER_CHECK,
	BOOT_TIMER_BEFORE_HANDOFF,
	BOOT_TIMER_PHASE_COUNT,
};

struct boot_timer_observation {
	uint32_t control_before;
	uint32_t count_before;
	uint32_t count_after;
	uint32_t control_after;
};

void boot_timer_observe_capture(struct boot_timer_observation *points, enum boot_timer_phase phase);
void boot_timer_observe_dump(const struct boot_timer_observation *points);

#ifdef BES_BOOT_TIMER_OBSERVE
#define BOOT_TIMER_DECLARE(name)        struct boot_timer_observation name[BOOT_TIMER_PHASE_COUNT]
#define BOOT_TIMER_CAPTURE(name, phase) boot_timer_observe_capture(name, phase)
#define BOOT_TIMER_DUMP(name)           boot_timer_observe_dump(name)
#else
#define BOOT_TIMER_DECLARE(name)
#define BOOT_TIMER_CAPTURE(name, phase) ((void)0)
#define BOOT_TIMER_DUMP(name)           ((void)0)
#endif

#endif
