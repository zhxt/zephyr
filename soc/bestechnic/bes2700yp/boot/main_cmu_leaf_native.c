/* SPDX-License-Identifier: Apache-2.0 */

#include "main_cmu_leaf_native.h"
#include "cmu_gate_native.h"

__attribute__((section(".boot_text_flash.hal_cmu_sys_m55_itcm_cfg"), noinline))
void hal_cmu_sys_m55_itcm_cfg(uint32_t enabled)
{
	uint32_t value = boot_cmu_gate_read(0x400800dcU);

	value &= ~0x7e00U;
	if (enabled != 0U) {
		value |= 0x1200U;
	}
	boot_cmu_gate_write(0x400800dcU, value);
	value = boot_cmu_gate_read(0x400800e0U);
	boot_cmu_gate_write(0x400800e0U, value);
}

__attribute__((section(".boot_text_flash.hal_cmu_bth_ram_cfg"), noinline))
void hal_cmu_bth_ram_cfg(void)
{
	/* The pinned selected-build interface is exactly BX LR. */
}

__attribute__((section(".boot_text_flash.hal_cmu_boot_dcdc_clock_enable"), noinline))
void hal_cmu_boot_dcdc_clock_enable(void)
{
	boot_cmu_gate_write(0x40080004U, 0x200U);
}

__attribute__((section(".boot_text_flash.hal_cmu_boot_dcdc_clock_disable"), noinline))
void hal_cmu_boot_dcdc_clock_disable(void)
{
	boot_cmu_gate_write(0x40080008U, 0x200U);
}

__attribute__((section(".boot_text_flash.hal_cmu_save_subsys_metal_id"), noinline))
void hal_cmu_save_subsys_metal_id(uint32_t revision)
{
	uint32_t value = boot_cmu_gate_read(0x400800f8U);

	boot_cmu_gate_write(0x400800f8U, (value & ~0xffU) | (revision & 0xffU));
}

void save_hw_metal_id(uint32_t revision)
	__attribute__((alias("hal_cmu_save_subsys_metal_id")));

__attribute__((section(".text.hal_cmu_reset_osc_clock_sel"), noinline))
void hal_cmu_reset_osc_clock_sel(void)
{
	/* The pinned selected-build interface is exactly BX LR. */
}
