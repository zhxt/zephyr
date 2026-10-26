/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_CMU_PERIPH_NATIVE_H_
#define BES_BOOT_CMU_PERIPH_NATIVE_H_

#include <stdint.h>

/* Fixed selected-build selectors; these values are not absolute frequencies. */
int hal_cmu_uart0_set_freq(uint32_t selector);
int hal_cmu_ispi_set_freq(uint32_t selector);
void boot_cmu_uart_modify(uint32_t clear_mask, uint32_t set_mask);

#endif /* BES_BOOT_CMU_PERIPH_NATIVE_H_ */
