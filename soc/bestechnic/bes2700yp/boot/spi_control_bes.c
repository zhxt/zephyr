/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>

#include "spi_control_native.h"

__attribute__((section(".text.boot_spi_read32"), noinline))
uint32_t boot_spi_read32(uintptr_t address)
{
	return sys_read32(address);
}

__attribute__((section(".text.boot_spi_write32"), noinline))
void boot_spi_write32(uintptr_t address, uint32_t value)
{
	sys_write32(value, address);
}
