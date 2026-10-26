/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_CMU_RESET_NATIVE_H_
#define BES_BOOT_CMU_RESET_NATIVE_H_

#include <stdint.h>

int hal_cmu_reset_set(uint32_t module);
int hal_cmu_reset_clear(uint32_t module);
void boot_cmu_reset_nops(uint32_t count);
void hal_cmu_reset_osc_clock_sel(void);

#endif /* BES_BOOT_CMU_RESET_NATIVE_H_ */
