/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>
#include <zephyr/arch/common/sys_bitops.h>

#include "psc_native.h"

/* Fixed-build bus addresses; public BES power/register semantics are unresolved. */
#ifdef PSC_MMIO_MODEL
extern volatile uint32_t *boot_psc_model_address(uint32_t address);
#define PSC_ADDRESS(address) ((uintptr_t)boot_psc_model_address(address))
#else
#define PSC_ADDRESS(address) ((uintptr_t)(address))
#endif

__attribute__((section(".boot_text_sram.boot_psc_read32")))
uint32_t boot_psc_read32(uint32_t address)
{
	return sys_read32(PSC_ADDRESS(address));
}

__attribute__((section(".boot_text_sram.boot_psc_write32")))
void boot_psc_write32(uint32_t address, uint32_t value)
{
	sys_write32(value, PSC_ADDRESS(address));
}
