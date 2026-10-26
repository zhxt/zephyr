/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_TIMER_NATIVE_H_
#define BES_BOOT_TIMER_NATIVE_H_

#include <stdint.h>

/* Fixed early-boot ABI; these interfaces do not enter a scheduler. */
void hal_sys_timer_open(void);
uint32_t hal_sys_timer_get(void);
uint32_t hal_sys_timer_get_in_sleep(void);
void hal_sys_timer_delay(uint32_t ticks);
void hal_sys_timer_delay_in_sleep(uint32_t ticks);
void hal_sys_timer_delay_us(uint32_t us);
int osDelay(uint32_t ms);

uint32_t boot_timer_slow_read32(void);
uint32_t boot_timer_fast_read32(void);
void boot_timer_hw_open(void);

/* Retained CMU leaf ABI. The initializer's integer return is ignored. */
int hal_cmu_fast_timer_init(void);
void hal_cmu_timer1_select_fast(void);
void hal_cmu_timer2_select_fast(void);

#endif /* BES_BOOT_TIMER_NATIVE_H_ */
