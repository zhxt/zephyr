/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_CMU_PLL_NATIVE_H_
#define BES_BOOT_CMU_PLL_NATIVE_H_

#include <stdint.h>

int hal_cmu_pll_enable(uint32_t pll, uint32_t user);
int hal_cmu_pll_disable(uint32_t pll, uint32_t user);
void hal_cmu_init_pll_selection(void);
void hal_cmu_init_axi_pll_selection(void);

/* Early boot state: initialize before CMU setup and preserve across normal CRT. */
extern volatile uint16_t boot_cmu_pll_users[3];

#endif /* BES_BOOT_CMU_PLL_NATIVE_H_ */
