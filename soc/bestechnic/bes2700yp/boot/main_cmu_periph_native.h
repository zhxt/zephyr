/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_MAIN_CMU_PERIPH_NATIVE_H_
#define BES_BOOT_MAIN_CMU_PERIPH_NATIVE_H_

#include <stdint.h>

/* Fixed selected-build selectors, without an implied absolute clock rate. */
int hal_cmu_spi_set_freq(uint32_t selector);
int hal_cmu_slcd_set_freq(uint32_t selector);

#endif /* BES_BOOT_MAIN_CMU_PERIPH_NATIVE_H_ */
