/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pmu_boot_native.h"

uint16_t boot_pmu_boot_reason __attribute__((section(".boot_bss.boot_pmu_boot_reason")));
uint8_t boot_pmu_boot_first_pwr_up __attribute__((section(".boot_bss.boot_pmu_boot_first_pwr_up")));
uint16_t boot_pmu_chg_ctx __attribute__((section(".boot_bss.boot_pmu_chg_ctx")));
