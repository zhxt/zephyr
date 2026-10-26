/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "spi_config_native.h"
#include "boot_diagnostics.h"

void boot_spi_config_fail(uint32_t reason, uint32_t first, uint32_t second)
{
	hal_trace_assert_dump("SPI config failure=%u arg0=%u arg1=%u", reason, first, second);
}
