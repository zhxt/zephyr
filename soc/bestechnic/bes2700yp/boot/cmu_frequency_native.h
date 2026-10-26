/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_CMU_FREQUENCY_NATIVE_H_
#define BES_BOOT_CMU_FREQUENCY_NATIVE_H_

#include <stdint.h>

uint32_t hal_cmu_sys_get_freq(void);
int hal_cmu_sys_set_freq(uint32_t frequency);
void hal_cmu_osc_x2_enable(void);
void hal_cmu_osc_x4_enable(void);
uint32_t boot_cmu_frequency_read(uintptr_t address);

/* Initialize before CMU setup and preserve across normal CRT initialization. */
extern volatile uint8_t boot_cmu_force_sys_div;

#endif /* BES_BOOT_CMU_FREQUENCY_NATIVE_H_ */
