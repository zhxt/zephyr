/* SPDX-License-Identifier: Apache-2.0 */

#include "cmu_setup_native.h"
#include "flash_readonly.h"

_Static_assert(sizeof(int) == sizeof(uint32_t), "fixed peripheral status ABI");

__attribute__((section(".boot_text_flash.hal_cmu_setup")))
void hal_cmu_setup(void)
{
	hal_iomux_set_default_config();
	hal_sys_timer_open();
	hal_hw_bootmode_init();
	hal_norflash_set_boot_freq(6U);
	(void)hal_cmu_mem_set_freq(6U);
	(void)hal_cmu_sys_set_freq(6U);
	hal_cmu_module_init_state();
	(void)hal_cmu_ispi_set_freq(0U);
	if (hal_analogif_open() != 0) {
		boot_clock_fault_halt();
	}
	hal_chipid_init();
	pmu_boot_init();
	hal_cmu_osc_x2_enable();
	hal_cmu_osc_x4_enable();
	while (hal_cmu_lpu_init(1U) == -1) {
	}
	hal_cmu_init_pll_selection();
	(void)hal_sysfreq_req(0U, 14U);
	/* The adapter checks the recorded result once UART diagnostics are ready. */
	(void)boot_flash_readonly_prepare(0U);
}
