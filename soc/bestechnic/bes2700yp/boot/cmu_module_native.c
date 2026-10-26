/* SPDX-License-Identifier: Apache-2.0 */

#include "cmu_module_native.h"
#include "cmu_gate_native.h"
#include "psc_native.h"
#include "timer_native.h"

__attribute__((section(".boot_text_flash.hal_cmu_module_init_state"), noinline))
void hal_cmu_module_init_state(void)
{
	uint32_t value;

	hal_psc_core_init_ram_ret_mask();
	value = boot_cmu_gate_read(0x400000f0U);
	boot_cmu_gate_write(0x400000f0U, (value & 0xfU) | 0x30010U);
	boot_cmu_gate_write(0x400000e0U, 0x1704034cU);
	boot_cmu_gate_write(0x400000e4U, 0x0b289210U);
	boot_cmu_gate_write(0x400000e8U, 0x0e491593U);
	value = boot_cmu_gate_read(0x400000ecU);
	boot_cmu_gate_write(0x400000ecU, (value & ~0x3fU) | 0xfU);
	boot_cmu_gate_write(0x40000064U, 0x7e000U);
	boot_cmu_gate_write(0x40000044U, 0x7d4U);
	boot_cmu_gate_write(0x4000003cU, 0x3eaU);
	boot_cmu_gate_write(0x40000034U, 0xe00U);
	boot_cmu_gate_write(0x40000014U, 0x7d4U);
	boot_cmu_gate_write(0x4000000cU, 0x3eaU);
	boot_cmu_gate_write(0x40000004U, 0xe00U);
	value = boot_cmu_gate_read(0x4000001cU);
	boot_cmu_gate_write(0x4000001cU, value & ~0xfU);
	value = boot_cmu_gate_read(0x40000068U);
	boot_cmu_gate_write(0x40000068U, (value & ~0x3000U) | 0x2000U);
	hal_cmu_main_module_init();
}

__attribute__((section(".boot_text_flash.hal_cmu_lpu_init"), noinline))
int hal_cmu_lpu_init(uint32_t mode)
{
	uint32_t first = 0x40U;
	uint32_t last = 0x740U;
	uint32_t value;

	if (mode > 2U) {
		return 1;
	}
	if ((boot_cmu_gate_read(0x4000008cU) & 0x5000U) == 0x4000U) {
		return -1;
	}
	/* Keep the second status snapshot independent of the first read. */
	if ((boot_cmu_gate_read(0x4000008cU) & 0xa000U) == 0x8000U) {
		return -1;
	}
	if (mode != 0U) {
		first = mode == 1U ? 0x1040U : 0x3040U;
		last = mode == 1U ? 0x1740U : 0x3740U;
		value = boot_cmu_gate_read(0x40000054U);
		boot_cmu_gate_write(0x40000054U, value & ~0x200U);
		value = boot_cmu_gate_read(0x40000018U);
		boot_cmu_gate_write(0x40000018U, value & ~0x7f003U);
		value = boot_cmu_gate_read(0x40080018U);
		boot_cmu_gate_write(0x40080018U, value | 0x1000U);
	} else {
		value = boot_cmu_gate_read(0x40080018U);
		boot_cmu_gate_write(0x40080018U, value & ~0x1000U);
	}
	value = boot_cmu_gate_read(0x40080018U);
	boot_cmu_gate_write(0x40080018U, (value & ~0x3fc000U) | 0x4d00000U);
	boot_cmu_gate_write(0x4000008cU, first);
	hal_sys_timer_delay(2U);
	boot_cmu_gate_write(0x4000008cU, last);
	return 0;
}
