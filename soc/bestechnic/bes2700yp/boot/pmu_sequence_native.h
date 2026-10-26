/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_PMU_SEQUENCE_NATIVE_H_
#define BOOT_PMU_SEQUENCE_NATIVE_H_

#include <stdint.h>

extern uint8_t boot_pmu_power_mode;
extern uint8_t boot_ana_act_dcdc, boot_ana_lp_dcdc;
extern uint8_t boot_dig_lp_dcdc, boot_dig_lp_ldo;

void pmu_dig_set_volt(uint32_t mode);
void pmu_mode_change(uint32_t mode);
void pmu_sys_freq_config(uint32_t frequency);

#endif /* BOOT_PMU_SEQUENCE_NATIVE_H_ */
