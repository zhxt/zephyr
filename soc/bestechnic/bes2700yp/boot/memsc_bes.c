/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>
#include <zephyr/arch/common/sys_bitops.h>

#include "memsc_native.h"

/* Fixed-build bus mapping; ownership side effects lack a public BES contract. */
#ifdef MEMSC_MMIO_MODEL
extern volatile uint32_t memsc_model_slots[];
#define MEMSC_SLOTS ((uintptr_t)memsc_model_slots)
#else
#define MEMSC_SLOTS ((uintptr_t)0x40080100U)
#endif

__attribute__((section(".boot_text_sram.boot_memsc_read32")))
uint32_t boot_memsc_read32(uint32_t offset)
{
	return sys_read32(MEMSC_SLOTS + offset);
}

__attribute__((section(".boot_text_sram.boot_memsc_write32")))
void boot_memsc_write32(uint32_t offset, uint32_t value)
{
	sys_write32(value, MEMSC_SLOTS + offset);
}
