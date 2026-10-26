/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_MAIN_CMU_MODULE_NATIVE_H_
#define BES_BOOT_MAIN_CMU_MODULE_NATIVE_H_

#include <stdint.h>

void hal_cmu_sys_module_init(void);
void hal_cmu_main_module_init(void);
void hal_cmu_init_axi_pll_selection(void);

#endif /* BES_BOOT_MAIN_CMU_MODULE_NATIVE_H_ */
