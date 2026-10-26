/* SPDX-License-Identifier: Apache-2.0 */

#include "cmu_release_native.h"
#include "cmu_gate_native.h"

__attribute__((section(".boot_text_flash.hal_cmu_boot_start_bth_cpu"), noinline))
void hal_cmu_boot_start_bth_cpu(uint32_t stack, uint32_t entry)
{
	uint32_t descriptor = (stack - 128U) & ~127U;
	uint32_t value = boot_cmu_gate_read(0x400800d8U);

	boot_cmu_gate_write(0x400800d8U, (value & 127U) | descriptor);
	boot_cmu_gate_write(descriptor, stack);
	boot_cmu_gate_write(descriptor + 4U, entry);
	boot_cmu_release_sync();
	/* Keep the descriptor readback between DSB and the release command. */
	(void)boot_cmu_gate_read(descriptor + 4U);
	boot_cmu_gate_write(0x400800ccU, 0x100U);
	boot_cmu_release_wait();
}

__attribute__((section(".boot_text_flash.hal_cmu_boot_m55_start_bth"), noinline))
void hal_cmu_boot_m55_start_bth(void)
{
	uint32_t continuation = (uint32_t)(uintptr_t)__builtin_return_address(0);

	hal_cmu_bth_ram_cfg();
	hal_cmu_boot_start_bth_cpu((uint32_t)(uintptr_t)&__StackTop, continuation | 1U);
}

void boot_loader_entry_hook(void)
	__attribute__((weak, alias("hal_cmu_boot_m55_start_bth"), noreturn));
