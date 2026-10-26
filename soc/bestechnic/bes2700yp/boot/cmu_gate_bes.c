/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>

#include "cmu_gate_native.h"

#ifdef CMU_GATE_MMIO_MODEL
uintptr_t cmu_gate_model_address(uintptr_t address);
#define GATE_ADDRESS(address) cmu_gate_model_address(address)
#else
#define GATE_ADDRESS(address) (address)
#endif

__attribute__((section(".boot_text_flash.boot_cmu_gate_read"), noinline))
uint32_t boot_cmu_gate_read(uintptr_t address)
{
	return sys_read32(GATE_ADDRESS(address));
}

__attribute__((section(".boot_text_flash.boot_cmu_gate_write"), noinline))
void boot_cmu_gate_write(uintptr_t address, uint32_t value)
{
	sys_write32(value, GATE_ADDRESS(address));
}
