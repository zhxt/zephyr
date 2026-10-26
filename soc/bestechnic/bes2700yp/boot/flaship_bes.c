/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>

#include "flaship_native.h"

#ifdef FLASHIP_MMIO_MODEL
uintptr_t flaship_model_address(uintptr_t address);
#define FLASHIP_ADDRESS(address) flaship_model_address(address)
#else
#define FLASHIP_ADDRESS(address) (address)
#endif

__attribute__((section(".boot_text_sram.boot_flaship_write8"),
	       noinline)) void boot_flaship_write8(uintptr_t address, uint8_t value)
{
	sys_write8(value, FLASHIP_ADDRESS(address));
}
