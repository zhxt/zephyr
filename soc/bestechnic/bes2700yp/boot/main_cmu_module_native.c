/* SPDX-License-Identifier: Apache-2.0 */

#include "main_cmu_module_native.h"
#include "main_cmu_axi_native.h"
#include "main_cmu_leaf_native.h"
#include "cmu_gate_native.h"
#include "cmu_timer_native.h"
#include "cmu_release_native.h"
#include "psc_native.h"
#include "timer_native.h"

static inline __attribute__((always_inline))
void modify(uintptr_t address, uint32_t clear, uint32_t set)
{
	uint32_t value = boot_cmu_gate_read(address);

	boot_cmu_gate_write(address, (value & ~clear) | set);
}

static inline __attribute__((always_inline)) void configure_axi_pll(uint32_t caller)
{
	uint32_t key = boot_cmu_timer_irq_lock(caller);

	modify(0x50000070U, 0x3000U, 0U);
	boot_cmu_timer_irq_unlock(key);
	/* Keep the two lock/read/write/unlock transactions independent. */
	key = boot_cmu_timer_irq_lock(caller);
	modify(0x5000010cU, 0x30000U, 0x20000U);
	boot_cmu_timer_irq_unlock(key);
}

__attribute__((section(".boot_text_flash.hal_cmu_sys_module_init"), noinline))
void hal_cmu_sys_module_init(void)
{
	const uintptr_t base = 0x50000000U;

	boot_cmu_gate_write(base + 0x1a8U, 0x40U);
	boot_cmu_gate_write(base + 0xd4U, 0x028e2040U);
	boot_cmu_gate_write(base + 0xd8U, 0x07185103U);
	boot_cmu_gate_write(base + 0xdcU, 0x18224261U);
	modify(base + 0x68U, 0x3fU, 0x19U);
	boot_cmu_gate_write(base + 0xe0U, 0x103ce34cU);
	boot_cmu_gate_write(base + 0xe4U, 0x1349d711U);
	boot_cmu_gate_write(base + 0xe8U, 0x206da554U);
	modify(base + 0xecU, 0x3fU, 0x24U);
	for (;;) {
		if ((boot_cmu_gate_read(base + 0x8cU) & 0x5000U) == 0x1000U) {
			continue;
		}
		if ((boot_cmu_gate_read(base + 0x8cU) & 0xa000U) != 0x2000U) {
			break;
		}
	}
	modify(base + 0x54U, 0x200U, 0U);
	modify(base + 0x18U, 0x21U, 0U);
	modify(base + 0x20U, 0x0fc00000U, 0U);
	modify(base + 0x180U, 0x1f801U, 0U);
	boot_cmu_gate_write(base + 0x8cU, 0x1040U);
	hal_sys_timer_delay(2U);
	boot_cmu_gate_write(base + 0x8cU, 0x1740U);
	boot_cmu_gate_write(base + 0x60U, 0x300U);
	boot_cmu_gate_write(base + 0x64U, 0xc40U);
	boot_cmu_gate_write(base + 0x64U, 0x1000U);
	boot_cmu_gate_write(base + 0x64U, 0x20U);
	boot_cmu_release_sync();
	boot_cmu_gate_write(base + 0x64U, 0x7e000U);
	boot_cmu_gate_write(base + 0x188U, 0x205efU);
	boot_cmu_gate_write(base + 0x170U, 0xfff8U);
	boot_cmu_gate_write(base + 0x44U, 0x1fffdfdcU);
	boot_cmu_gate_write(base + 0x3cU, 0x0f41fbfaU);
	boot_cmu_gate_write(base + 0x34U, 0x5ff80U);
	boot_cmu_gate_write(base + 0x17cU, 0x205efU);
	boot_cmu_gate_write(base + 0x164U, 0xfff8U);
	boot_cmu_gate_write(base + 0x14U, 0x1fffdfdcU);
	boot_cmu_gate_write(base + 0xcU, 0x0f41fbfaU);
	boot_cmu_gate_write(base + 4U, 0x5ff80U);
	boot_cmu_gate_write(base + 0x34U, 0x80000U);
	boot_cmu_gate_write(base + 4U, 0x80000U);
	modify(base + 0x1cU, 0x20003fU, 0U);
	modify(base + 0x194U, 0xcU, 4U);
	modify(base + 0x194U, 0x13U, 3U);
	modify(base + 0xf8U, 0U, 0x40004U);
	boot_cmu_gate_write(base + 0x150U, 0U);
	modify(base + 0x150U, 0U, 0x800000U);
	modify(base + 0x19cU, 0x100000U, 0U);
}

__attribute__((section(".boot_text_flash.hal_cmu_main_module_init"), noinline))
void hal_cmu_main_module_init(void)
{
	const uintptr_t aon = 0x40080000U;
	const uintptr_t base = 0x50000000U;
	uint32_t caller = (uint32_t)(uintptr_t)__builtin_return_address(0);
	uint32_t value;
	uint32_t selection;

	modify(aon + 0x4cU, 0U, 1U);
	boot_cmu_gate_write(aon + 8U, 4U);
	modify(aon + 0x1cU, 0x1f00U, 0x200U);
	boot_cmu_gate_write(aon + 0x88U, 0x3ff00000U);
	boot_cmu_gate_write(aon + 0x90U, 0x3ff00000U);
	boot_cmu_gate_write(aon + 0xb0U, 0x3ff00000U);
	boot_cmu_gate_write(aon + 0xe8U, 0x3ff00000U);
	boot_cmu_gate_write(aon + 0x24U, 0x3ff00000U);
	boot_cmu_gate_write(aon + 0x2cU, 0x3ff00000U);
	boot_cmu_gate_write(aon + 0x16cU, 0x3ff00000U);
	boot_cmu_gate_write(aon + 8U, 0x200U);
	modify(aon + 0xdcU, 0x3fff8000U, 0x1b6d8000U);
	modify(aon + 0xe0U, 0U, 0U);
	hal_cmu_sys_m55_itcm_cfg(1U);
	/* The original repeats both transactions after the ITCM callback. */
	modify(aon + 0xdcU, 0U, 0U);
	modify(aon + 0xe0U, 0U, 0U);
	modify(aon + 0x98U, 0x3f0000U, 0x2a0000U);
	(void)boot_cmu_gate_read(aon);
	(void)boot_cmu_gate_read(aon);
	boot_cmu_gate_write(aon + 0x10U, 0x10000U);
	boot_cmu_gate_write(aon + 0x44U, 0x10000U);
	boot_cmu_gate_write(aon + 0xc8U, 0x24U);
	boot_cmu_gate_write(aon + 0xa4U, 0x1000U);
	boot_cmu_gate_write(aon + 0xc8U, 0x10U);
	hal_cmu_axi_clock_enable(2U);
	hal_cmu_axi_reset_clear(2U);
	boot_cmu_gate_write(base + 0x60U, 0x2000U);
	boot_cmu_gate_write(base + 0x60U, 0x8000U);
	hal_cmu_sys_module_init();
	boot_cmu_gate_write(base + 0x60U, 0x4000U);
	hal_sys_timer_delay(2U);
	boot_cmu_gate_write(base + 0x60U, 0x2000U);
	configure_axi_pll(caller);
	boot_cmu_gate_write(aon + 0xa4U, UINT32_MAX);
	boot_cmu_gate_write(aon + 8U, 0xcfd37U);
	boot_cmu_gate_write(aon + 0xc8U, 0x24U);
	boot_cmu_gate_write(aon + 0x48U, 0xfffff786U);
	value = boot_cmu_gate_read(aon);
	selection = value < 0x30000U ? 0U : 3U;
	modify(aon + 0xd0U, 7U, selection);
	modify(aon + 0xd0U, 0x70000U, selection << 16U);
	hal_psc_init();
}

__attribute__((section(".boot_text_flash.hal_cmu_init_axi_pll_selection"), noinline))
void hal_cmu_init_axi_pll_selection(void)
{
	configure_axi_pll((uint32_t)(uintptr_t)__builtin_return_address(0));
}
