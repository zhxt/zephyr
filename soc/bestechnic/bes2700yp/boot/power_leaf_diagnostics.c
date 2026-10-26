/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "power_leaf_native.h"
#include "boot_diagnostics.h"

void boot_power_leaf_fail(uint32_t operation, uint32_t address, int status)
{
	hal_trace_assert_dump("Power transfer operation=%u address=%x status=%d",
			      operation, address, status);
}
