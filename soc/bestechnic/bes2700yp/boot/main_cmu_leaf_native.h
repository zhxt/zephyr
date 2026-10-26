/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_MAIN_CMU_LEAF_NATIVE_H_
#define BES_BOOT_MAIN_CMU_LEAF_NATIVE_H_

#include <stdint.h>

/* Fixed selected-build interfaces; these names do not establish public register semantics. */
void hal_cmu_sys_m55_itcm_cfg(uint32_t enabled);
void hal_cmu_bth_ram_cfg(void);
void hal_cmu_boot_dcdc_clock_enable(void);
void hal_cmu_boot_dcdc_clock_disable(void);
uint32_t hal_cmu_get_aon_revision_id(void);
void hal_cmu_save_subsys_metal_id(uint32_t revision);
void save_hw_metal_id(uint32_t revision);
void hal_cmu_reset_osc_clock_sel(void);

#endif /* BES_BOOT_MAIN_CMU_LEAF_NATIVE_H_ */
