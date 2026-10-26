/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pmu_module_native.h"

_Static_assert(sizeof(struct boot_pmu_module_config) == 18U, "PMU descriptor ABI");
_Static_assert(sizeof(boot_pmu_module_cfg) == 198U, "PMU descriptor storage");

__attribute__((section(".boot_bss.boot_pmu_module_cfg"))) struct boot_pmu_module_config
	boot_pmu_module_cfg[BOOT_PMU_MODULE_COUNT];
