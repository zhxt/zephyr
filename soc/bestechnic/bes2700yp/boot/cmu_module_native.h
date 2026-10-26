/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_CMU_MODULE_NATIVE_H_
#define BES_BOOT_CMU_MODULE_NATIVE_H_

#include <stdint.h>

void hal_cmu_module_init_state(void);
int hal_cmu_lpu_init(uint32_t mode);
void hal_cmu_main_module_init(void);

#endif /* BES_BOOT_CMU_MODULE_NATIVE_H_ */
