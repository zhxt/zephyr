/* SPDX-License-Identifier: Apache-2.0 */

#include "memsc_native.h"

_Static_assert(sizeof(int) == sizeof(uint32_t), "fixed lock return ABI");

__attribute__((section(".boot_text_sram.hal_memsc_aon_lock")))
int hal_memsc_aon_lock(uint32_t selector)
{
	if (selector >= BOOT_MEMSC_SLOT_COUNT) {
		return 0;
	}
	return (int)boot_memsc_read32(selector * BOOT_MEMSC_SLOT_STRIDE);
}

__attribute__((section(".boot_text_sram.hal_memsc_aon_unlock")))
void hal_memsc_aon_unlock(uint32_t selector)
{
	if (selector < BOOT_MEMSC_SLOT_COUNT) {
		boot_memsc_write32(selector * BOOT_MEMSC_SLOT_STRIDE, 1U);
	}
}
