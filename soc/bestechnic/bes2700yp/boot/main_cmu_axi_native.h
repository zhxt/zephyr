/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_MAIN_CMU_AXI_NATIVE_H_
#define BES_BOOT_MAIN_CMU_AXI_NATIVE_H_

#include <stdint.h>

/* Fixed selected-build user numbers; no public AXI/register semantics are implied. */
void hal_cmu_axi_clock_enable(uint32_t user);
void hal_cmu_axi_reset_clear(uint32_t user);
extern uint8_t boot_main_cmu_axi_clk_map;
extern uint8_t boot_main_cmu_axi_rst_map;

#endif /* BES_BOOT_MAIN_CMU_AXI_NATIVE_H_ */
