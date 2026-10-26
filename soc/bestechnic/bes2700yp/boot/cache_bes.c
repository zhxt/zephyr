/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>
#include <zephyr/arch/common/sys_bitops.h>

#include "cache_native.h"

/* Fixed-build experiment mapping, without a public BES cache specification. */
#ifdef CACHE_MMIO_MODEL
extern volatile uint32_t cache_model_registers[];
#define CACHE_BASE ((uintptr_t)cache_model_registers)
#else
#define CACHE_BASE ((uintptr_t)0x07ffa000U)
#endif

__attribute__((section(".boot_text_flash.boot_cache_early_read32")))
uint32_t boot_cache_early_read32(uint32_t offset)
{
	return sys_read32(CACHE_BASE + offset);
}

__attribute__((section(".boot_text_flash.boot_cache_early_write32")))
void boot_cache_early_write32(uint32_t offset, uint32_t value)
{
	sys_write32(value, CACHE_BASE + offset);
}

__attribute__((section(".boot_text_sram.boot_cache_read32")))
uint32_t boot_cache_read32(uint32_t offset)
{
	return sys_read32(CACHE_BASE + offset);
}

__attribute__((section(".boot_text_sram.boot_cache_write32")))
void boot_cache_write32(uint32_t offset, uint32_t value)
{
	sys_write32(value, CACHE_BASE + offset);
}
