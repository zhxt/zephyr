/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BES_BOOT_TIMER_DOMAINS_OBSERVE_H_
#define BES_BOOT_TIMER_DOMAINS_OBSERVE_H_

#include "timer_observe.h"

struct boot_timer_domains_point {
	uint32_t control_before;
	uint32_t slow_before;
	uint32_t fast_before;
	uint32_t slow_after;
	uint32_t fast_after;
	uint32_t control_after;
	uint32_t primask_before;
	uint32_t basepri_before;
	uint32_t primask_after;
	uint32_t basepri_after;
};

void boot_timer_domains_capture(struct boot_timer_domains_point *points,
				enum boot_timer_phase phase);
void boot_timer_domains_dump(const struct boot_timer_domains_point *points);

#ifdef BES_BOOT_TIMER_DOMAINS_OBSERVE
#define BOOT_TD_DECLARE(name) \
	struct boot_timer_domains_point name[BOOT_TIMER_PHASE_COUNT]
#define BOOT_TD_CAPTURE(name, phase) boot_timer_domains_capture(name, phase)
#define BOOT_TD_DUMP(name) boot_timer_domains_dump(name)
#else
#define BOOT_TD_DECLARE(name)
#define BOOT_TD_CAPTURE(name, phase) ((void)0)
#define BOOT_TD_DUMP(name) ((void)0)
#endif

#endif /* BES_BOOT_TIMER_DOMAINS_OBSERVE_H_ */
