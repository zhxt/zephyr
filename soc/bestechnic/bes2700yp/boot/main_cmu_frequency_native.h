/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_MAIN_CMU_FREQUENCY_NATIVE_H_
#define BES_BOOT_MAIN_CMU_FREQUENCY_NATIVE_H_

#include <stdint.h>

int hal_cmu_mem_set_freq(uint32_t frequency);
int hal_cmu_sys_flash_set_freq(uint32_t frequency);
int hal_cmu_flash1_set_freq(uint32_t frequency);
int hal_cmu_bth_flash_set_freq(uint32_t frequency);
int hal_cmu_flash_set_freq(uint32_t frequency);
uint32_t boot_main_cmu_frequency_read(uintptr_t address);
void boot_main_cmu_frequency_write(uintptr_t address, uint32_t value);
uint32_t boot_main_cmu_frequency_irq_lock(uint32_t caller);
void boot_main_cmu_frequency_irq_unlock(uint32_t key);
void boot_main_cmu_frequency_barrier(void);

#endif /* BES_BOOT_MAIN_CMU_FREQUENCY_NATIVE_H_ */
