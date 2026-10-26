/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>

#include "cmu_frequency_native.h"

#ifdef CMU_FREQUENCY_MMIO_MODEL
uintptr_t cmu_gate_model_address(uintptr_t address);
#define FREQUENCY_ADDRESS(address) cmu_gate_model_address(address)
#else
#define FREQUENCY_ADDRESS(address) (address)
#endif

__attribute__((section(".boot_text_sram.boot_cmu_frequency_read"), noinline))
uint32_t boot_cmu_frequency_read(uintptr_t address)
{
	return sys_read32(FREQUENCY_ADDRESS(address));
}
