/* SPDX-License-Identifier: Apache-2.0 */

#include "bootmode_native.h"

_Static_assert(sizeof(union boot_hw_bootmode) == 1, "fixed boot-mode return ABI");

__attribute__((section(".boot_bss.bootmode_native"))) static uint8_t saved_mode;

__attribute__((section(".boot_text_flash.hal_hw_bootmode_init")))
void hal_hw_bootmode_init(void)
{
	saved_mode = boot_mode_hw_capture();
	boot_mode_hw_acknowledge();
}

__attribute__((section(".boot_text_flash.hal_hw_bootmode_get")))
union boot_hw_bootmode hal_hw_bootmode_get(void)
{
	union boot_hw_bootmode mode = {.reg = saved_mode};

	return mode;
}
