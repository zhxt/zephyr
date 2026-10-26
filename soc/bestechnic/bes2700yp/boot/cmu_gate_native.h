/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_CMU_GATE_NATIVE_H_
#define BES_BOOT_CMU_GATE_NATIVE_H_

#include <stdint.h>

int hal_cmu_clock_enable(uint32_t module);
int hal_cmu_clock_disable(uint32_t module);
int hal_cmu_clock_get_status(uint32_t module);
uint32_t boot_cmu_gate_read(uintptr_t address);
void boot_cmu_gate_write(uintptr_t address, uint32_t value);

#endif /* BES_BOOT_CMU_GATE_NATIVE_H_ */
