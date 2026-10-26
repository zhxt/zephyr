/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pmu_sequence_native.h"

__attribute__((section(".boot_bss.boot_pmu_power_mode"))) uint8_t boot_pmu_power_mode;
uint8_t boot_ana_act_dcdc, boot_ana_lp_dcdc;
uint8_t boot_dig_lp_dcdc, boot_dig_lp_ldo;
