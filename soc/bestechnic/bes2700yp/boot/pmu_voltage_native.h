/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_PMU_VOLTAGE_NATIVE_H_
#define BOOT_PMU_VOLTAGE_NATIVE_H_

#include <stdint.h>

extern uint8_t boot_pmu_type, boot_pmu_metal_id;
extern uint32_t boot_pmu_vcore_req;
extern uint16_t boot_pmu_dcdc_ramp_map;
extern int8_t boot_pmu_ana_lp_comp, boot_pmu_dcdc_ana_comp;
extern int8_t boot_pmu_dig_lp_comp, boot_pmu_dcdc_dig_comp;
extern int8_t boot_pmu_hppa_lp_comp, boot_pmu_dcdc_hppa_comp;
extern int8_t boot_pmu_ldo_dig_m_comp, boot_pmu_ldo_dig_l_comp;
extern int8_t boot_pmu_ldo_lp_m_comp, boot_pmu_ldo_lp_l_comp;

void pmu_dig_get_target_volt(uint16_t *ldo, uint16_t *dcdc_low, uint16_t *dcdc);
void pmu_dig_init_volt(void);
uint32_t read_hw_metal_id(void);
uint32_t pmu_get_chip_type(void);
void pmu_dcdc_ana_set_volt(uint32_t normal, uint32_t low);
void pmu_dcdc_dig_set_volt(uint32_t normal, uint32_t low);
void pmu_dcdc_hppa_set_volt(uint32_t normal, uint32_t low);
void pmu_get_ldo_dig_calib_value(void);
void pmu_codec_mic_bias_set_volt(uint32_t channels, uint32_t voltage);

#endif /* BOOT_PMU_VOLTAGE_NATIVE_H_ */
