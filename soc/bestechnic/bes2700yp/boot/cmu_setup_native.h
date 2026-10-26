/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_CMU_SETUP_NATIVE_H_
#define BES_BOOT_CMU_SETUP_NATIVE_H_

#include <stdint.h>

/* Fixed selected-build ABI. Numeric clock selectors remain opaque here. */
void hal_cmu_setup(void);
void hal_iomux_set_default_config(void);
void hal_sys_timer_open(void);
void hal_hw_bootmode_init(void);
void hal_norflash_set_boot_freq(uint32_t frequency);
int hal_cmu_mem_set_freq(uint32_t frequency);
int hal_cmu_sys_set_freq(uint32_t frequency);
void hal_cmu_module_init_state(void);
int hal_cmu_ispi_set_freq(uint32_t frequency);
int hal_analogif_open(void);
void hal_chipid_init(void);
void pmu_boot_init(void);
void hal_cmu_osc_x2_enable(void);
void hal_cmu_osc_x4_enable(void);
int hal_cmu_lpu_init(uint32_t configuration);
void hal_cmu_init_pll_selection(void);
int hal_sysfreq_req(uint32_t user, uint32_t frequency);

__attribute__((noreturn)) void boot_clock_fault_halt(void);

#endif /* BES_BOOT_CMU_SETUP_NATIVE_H_ */
