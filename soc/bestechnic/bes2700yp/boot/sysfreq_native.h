/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_SYSFREQ_NATIVE_H_
#define BES_BOOT_SYSFREQ_NATIVE_H_

#include <stdint.h>

/* Fixed selected-build ABI; selector values do not express MHz here. */
int hal_sysfreq_req(uint32_t user, uint32_t frequency);
uint32_t hal_cmu_sys_get_freq(void);
int hal_cmu_sys_set_freq(uint32_t frequency);
void pmu_sys_freq_config(uint32_t frequency);
uint32_t boot_sysfreq_irq_lock(uint32_t caller);
void boot_sysfreq_irq_unlock(uint32_t key);

extern volatile uint8_t boot_sysfreq_top_user;
extern volatile uint8_t boot_sysfreq_minimum;
extern volatile uint8_t boot_sysfreq_requests[44];

#endif /* BES_BOOT_SYSFREQ_NATIVE_H_ */
