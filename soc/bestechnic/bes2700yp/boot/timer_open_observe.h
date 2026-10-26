/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BES_BOOT_TIMER_OPEN_OBSERVE_H_
#define BES_BOOT_TIMER_OPEN_OBSERVE_H_

#include <stdint.h>

#define BOOT_TIMER_OPEN_CAPACITY 2U

struct boot_timer_open_point {
	uint32_t entry_primask;
	uint32_t entry_basepri;
	uint32_t primask_before;
	uint32_t basepri_before;
	uint32_t control_before;
	uint32_t fast_before;
	uint32_t fast_after;
	uint32_t control_after;
	uint32_t primask_after;
	uint32_t basepri_after;
};

struct boot_timer_open_state {
	uint32_t calls;
	uint32_t overflow;
	struct boot_timer_open_point points[BOOT_TIMER_OPEN_CAPACITY];
};

void __wrap_hal_sys_timer_open(void);
void __real_hal_sys_timer_open(void);
#ifndef BOOT_TIMER_OPEN_HOST
/* Bind the Flash-to-SRAM transfer without an unbound linker veneer. */
__attribute__((long_call))
#endif
void boot_timer_open_capture(struct boot_timer_open_point *point);
void boot_timer_open_dump(void);

#ifdef BES_BOOT_TIMER_OPEN_OBSERVE
#define BOOT_TIMER_OPEN_DUMP() boot_timer_open_dump()
#else
#define BOOT_TIMER_OPEN_DUMP() ((void)0)
#endif

#endif /* BES_BOOT_TIMER_OPEN_OBSERVE_H_ */
