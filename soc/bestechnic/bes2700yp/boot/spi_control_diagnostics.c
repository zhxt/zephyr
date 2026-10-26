/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "spi_control_native.h"
#include "boot_diagnostics.h"

void boot_spi_control_fail(uint32_t reason, uint32_t argument)
{
	hal_trace_assert_dump("SPI control failure=%u argument=%u", reason, argument);
}
