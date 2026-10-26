/* SPDX-License-Identifier: Apache-2.0 */

#include "chipid_native.h"

/* The normal CRT runs later, so this state belongs to early boot BSS. */
__attribute__((section(".boot_bss.chipid_native"))) uint8_t metal_id;

__attribute__((section(".boot_text_flash.hal_chipid_init")))
void hal_chipid_init(void)
{
	uint8_t revision = (uint8_t)read_hw_metal_id();

	metal_id = revision;
	save_hw_metal_id((uint32_t)revision);
}
