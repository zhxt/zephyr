/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>
#include <zephyr/arch/common/sys_bitops.h>

#include "iomux_native.h"

/* Fixed-build bus addresses; public BES pin/register semantics are unresolved. */
#ifdef IOMUX_MMIO_MODEL
extern volatile uint32_t *boot_iomux_model_address(uint32_t address);
#define IOMUX_ADDRESS(address) ((uintptr_t)boot_iomux_model_address(address))
#else
#define IOMUX_ADDRESS(address) ((uintptr_t)(address))
#endif

__attribute__((section(".boot_text_flash.boot_iomux_read32")))
uint32_t boot_iomux_read32(uint32_t address)
{
	return sys_read32(IOMUX_ADDRESS(address));
}

__attribute__((section(".boot_text_flash.boot_iomux_write32")))
void boot_iomux_write32(uint32_t address, uint32_t value)
{
	sys_write32(value, IOMUX_ADDRESS(address));
}
