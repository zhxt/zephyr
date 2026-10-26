/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pmu_voltage_native.h"

#define EARLY(name) __attribute__((section(".boot_bss." #name)))

EARLY(boot_pmu_type) uint8_t boot_pmu_type;
EARLY(boot_pmu_metal_id) uint8_t boot_pmu_metal_id;
EARLY(boot_pmu_vcore_req) uint32_t boot_pmu_vcore_req;
__attribute__((section(".sram_bss.boot_pmu_dcdc_ramp_map"))) uint16_t boot_pmu_dcdc_ramp_map;
int8_t boot_pmu_ana_lp_comp, boot_pmu_dcdc_ana_comp;
int8_t boot_pmu_dig_lp_comp, boot_pmu_dcdc_dig_comp;
int8_t boot_pmu_hppa_lp_comp, boot_pmu_dcdc_hppa_comp;
int8_t boot_pmu_ldo_dig_m_comp, boot_pmu_ldo_dig_l_comp;
int8_t boot_pmu_ldo_lp_m_comp, boot_pmu_ldo_lp_l_comp;
