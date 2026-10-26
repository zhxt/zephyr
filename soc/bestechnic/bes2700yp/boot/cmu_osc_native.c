/* SPDX-License-Identifier: Apache-2.0 */

#include "cmu_frequency_native.h"
#include "cmu_gate_native.h"
#include "timer_native.h"

__attribute__((section(".boot_text_flash.hal_cmu_osc_x2_enable"), noinline))
void hal_cmu_osc_x2_enable(void)
{
	boot_cmu_gate_write(0x40000060U, 0x4000U);
	hal_sys_timer_delay(2U);
	boot_cmu_gate_write(0x40000060U, 0x2000U);
}

__attribute__((section(".boot_text_flash.hal_cmu_osc_x4_enable"), noinline))
void hal_cmu_osc_x4_enable(void)
{
	/* The selected original build returns immediately, with no hardware access. */
}
