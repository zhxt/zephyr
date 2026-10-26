/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/sys/util.h>

#include "boot_diagnostics.h"
#include "itn_native.h"
#include "main_cmu_leaf_native.h"
#include "pmu_voltage_native.h"
#include "power_transfer.h"

#define SRAM(name)   __attribute__((section(".boot_text_sram." #name), noinline))
#define FLASH(name)  __attribute__((section(".boot_text_flash." #name), noinline))
#define NORMAL(name) __attribute__((section(".text." #name), noinline))

__attribute__((
	section(".boot_data.pmu_target_codes"))) static const uint16_t target_codes[4][5][3] = {
	{{24U, 176U, 192U},
	 {30U, 232U, 208U},
	 {24U, 152U, 160U},
	 {24U, 120U, 128U},
	 {24U, 104U, 112U}},
	{{25U, 152U, 26U}, {32U, 232U, 28U}, {25U, 152U, 22U}, {25U, 120U, 18U}, {25U, 104U, 16U}},
	{{20U, 69U, 24U}, {30U, 115U, 26U}, {20U, 76U, 20U}, {20U, 61U, 16U}, {20U, 53U, 14U}},
	{{20U, 144U, 192U},
	 {30U, 232U, 208U},
	 {20U, 152U, 160U},
	 {20U, 120U, 128U},
	 {20U, 104U, 112U}},
};

SRAM(pmu_dig_get_target_volt)
void pmu_dig_get_target_volt(uint16_t *ldo, uint16_t *dcdc_low, uint16_t *dcdc)
{
	uint32_t requested = boot_pmu_vcore_req;
	uint32_t type = boot_pmu_type <= 3U ? boot_pmu_type : 0U;
	uint32_t level = requested & 0x8000U    ? 0U
			 : requested & 0x4020U  ? 1U
			 : requested & 0x12057U ? 2U
			 : requested & 0x0fc0U  ? 3U
						: 4U;
	const uint16_t *codes = target_codes[type][level];

	*ldo = codes[0];
	if (dcdc_low != NULL) {
		*dcdc_low = codes[1];
	}
	*dcdc = codes[2];
}

FLASH(pmu_dig_init_volt)
void pmu_dig_init_volt(void)
{
	uint16_t target, unused;

	pmu_dig_get_target_volt(&target, NULL, &unused);
	uint16_t value = boot_power_read16(8U);

	if (target > (value & 0x3fU)) {
		boot_power_write16(8U, (value & ~0x3fU) | (target & 0x3fU));
	}
}

FLASH(read_hw_metal_id)
uint32_t read_hw_metal_id(void)
{
	uint16_t value = boot_power_read16(0U);

	switch (value >> 4) {
	case 0x183U:
		boot_pmu_type = 0U;
		break;
	case 0x185U:
		boot_pmu_type = 1U;
		break;
	case 0x186U:
		boot_pmu_type = 2U;
		break;
	case 0x188U:
		boot_pmu_type = 3U;
		break;
	default:
		break;
	}
	boot_pmu_metal_id = value & 15U;
	uint32_t revision = hal_cmu_get_aon_revision_id();

	return MAX(revision, 1U);
}

FLASH(pmu_get_chip_type)
uint32_t pmu_get_chip_type(void)
{
	return boot_pmu_type;
}

__attribute__((always_inline)) static inline uint32_t compensated(uint32_t code, int8_t adjustment)
{
	/* ARM ADD wraps before signed USAT. Avoid signed-overflow C arithmetic. */
	int32_t sum = (int32_t)(code + (uint32_t)(int32_t)adjustment);

	return (uint32_t)CLAMP(sum, 0, 255);
}

__attribute__((always_inline)) static inline void set_dcdc(uint32_t normal, uint32_t low,
							   uint32_t reg, uint32_t transition_reg,
							   uint16_t ramp_bit, int8_t active_comp,
							   int8_t low_comp)
{
	uint32_t active = compensated(normal, active_comp);
	uint32_t sleeping = compensated(low, low_comp);
	uint32_t transition = compensated(normal, low_comp);

	(void)boot_power_read16(reg);
	boot_power_write16(reg, active | (sleeping << 8));
	uint32_t type = boot_pmu_type;

	if (type == 1U || type == 2U) {
		uint16_t value = boot_power_read16(transition_reg);

		boot_power_write16(transition_reg, (value & ~255U) | transition);
	}
	if (sleeping < active || transition > sleeping) {
		boot_pmu_dcdc_ramp_map |= ramp_bit;
	} else {
		boot_pmu_dcdc_ramp_map &= (uint16_t)~ramp_bit;
	}
}

NORMAL(pmu_dcdc_ana_set_volt)
void pmu_dcdc_ana_set_volt(uint32_t normal, uint32_t low)
{
	set_dcdc(normal, low, 0x47U, 0x75U, 8U, boot_pmu_dcdc_ana_comp, boot_pmu_ana_lp_comp);
}

NORMAL(pmu_dcdc_dig_set_volt)
void pmu_dcdc_dig_set_volt(uint32_t normal, uint32_t low)
{
	set_dcdc(normal, low, 0x46U, 0x74U, 16U, boot_pmu_dcdc_dig_comp, boot_pmu_dig_lp_comp);
}

NORMAL(pmu_dcdc_hppa_set_volt)
void pmu_dcdc_hppa_set_volt(uint32_t normal, uint32_t low)
{
	set_dcdc(normal, low, 0x48U, 0x76U, 4U, boot_pmu_dcdc_hppa_comp, boot_pmu_hppa_lp_comp);
}

__attribute__((always_inline)) static inline uint16_t efuse(uint32_t page)
{
	uint16_t value = 0U;
	int result = pmu_get_efuse(page, &value);

	if (result != 0) {
		boot_power_leaf_fail(2U, page, result);
	}
	return value;
}

__attribute__((always_inline)) static inline int8_t calibration(uint16_t word, uint32_t shift)
{
	uint32_t field = (word >> shift) & 0x3fU;

	return (int8_t)((field & 0x20U) != 0U ? -(int32_t)(field & 31U) : (int32_t)field);
}

NORMAL(pmu_get_ldo_dig_calib_value)
void pmu_get_ldo_dig_calib_value(void)
{
	if (boot_pmu_type == 1U && boot_pmu_metal_id == 0U) {
		return;
	}
	uint16_t word = efuse(8U);

	boot_pmu_ldo_dig_m_comp = calibration(word, 6U);
	boot_pmu_ldo_dig_l_comp = calibration(word, 0U);
	word = efuse(10U);
	boot_pmu_ldo_lp_m_comp = calibration(word, 6U);
	boot_pmu_ldo_lp_l_comp = calibration(word, 0U);
	boot_pmu_ldo_dig_m_comp = 0;
	(void)hal_trace_printf(0x42U, "pmu_ldo_dig_m_comp:%d", 0);
	(void)hal_trace_printf(0x42U, "pmu_ldo_dig_l_comp:%d", boot_pmu_ldo_dig_l_comp);
	(void)hal_trace_printf(0x42U, "pmu_ldo_lp_m_comp:%d", boot_pmu_ldo_lp_m_comp);
	(void)hal_trace_printf(0x42U, "pmu_ldo_lp_l_comp:%d", boot_pmu_ldo_lp_l_comp);
}

NORMAL(pmu_codec_mic_bias_set_volt)
void pmu_codec_mic_bias_set_volt(uint32_t channels, uint32_t voltage)
{
	uint32_t type = boot_pmu_type;
	uint32_t count = type == 0U || type == 3U ? 3U : 2U;
	uint32_t step = type == 1U && boot_pmu_metal_id == 1U ? 120U : 100U;
	uint32_t half = step / 2U;
	uint32_t field;

	if (type == 0U) {
		if ((channels & 0x80000U) != 0U) {
			channels = (channels & ~0x180000U) | 0x100000U;
		}
		if ((channels & 0x400000U) != 0U) {
			channels = (channels & ~0x480000U) | 0x80000U;
		}
	}
	if (voltage <= 1600U) {
		field = type == 2U ? 0x100U : 0xe0U;
	} else if (voltage < 1900U + half) {
		field = (uint8_t)((uint8_t)((voltage - 1600U + half) / step) + 1U +
				  (type == 2U ? 7U : 6U));
		field <<= 5;
	} else if (voltage < 2500U + half) {
		field = (uint8_t)((uint8_t)((voltage - 2000U + half) / step) + 1U +
				  (type == 1U ? 9U : 10U));
		field <<= 5;
	} else {
		uint32_t code = (uint8_t)((uint8_t)((voltage - 2600U + half) / step) +
					  (type == 1U ? 16U : 20U));

		field = (code > 30U ? 31U : code + 1U) << 5;
	}
	for (uint32_t i = 0U; i < count; i++) {
		if ((channels & (0x40000U << i)) != 0U) {
			uint32_t reg = 0x3bU + i;
			uint16_t value = boot_power_read16(reg);

			boot_power_write16(reg, (value & 0xfc01U) | field | 0x1eU);
		}
	}
}
