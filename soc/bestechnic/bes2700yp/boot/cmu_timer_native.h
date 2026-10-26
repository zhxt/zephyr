/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_CMU_TIMER_NATIVE_H_
#define BES_BOOT_CMU_TIMER_NATIVE_H_

#include <stdint.h>

/* Fixed selected-build interfaces; no absolute timer frequency is implied. */
void hal_cmu_timer1_select_fast(void);
void hal_cmu_timer2_select_fast(void);
int hal_cmu_fast_timer_init(void);
void boot_cmu_timer_modify(uint32_t clear_mask, uint32_t set_mask);
uint32_t boot_cmu_timer_irq_lock(uint32_t caller);
void boot_cmu_timer_irq_unlock(uint32_t key);

#endif /* BES_BOOT_CMU_TIMER_NATIVE_H_ */
