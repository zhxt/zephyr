/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/sys/util.h>

#include "main_cmu_frequency_native.h"
#include "pmu_module_native.h"
#include "pmu_sequence_native.h"
#include "pmu_voltage_native.h"
#include "power_transfer.h"
#include "timer_native.h"

#define NORMAL(name) __attribute__((section(".text." #name), noinline))
#define SRAM(name)   __attribute__((section(".boot_text_sram." #name), noinline))
#define INLINE       __attribute__((always_inline)) static inline

INLINE uint16_t calibrated(uint32_t code, int8_t adjustment)
{
	return (uint16_t)CLAMP((int32_t)code - adjustment, 0, 255);
}

INLINE uint16_t update(uint32_t address, uint16_t clear, uint16_t set)
{
	uint16_t value = boot_power_read16(address);

	value = (value & (uint16_t)~clear) | set;
	boot_power_write16(address, value);
	return value;
}

NORMAL(pmu_dig_set_volt)
void pmu_dig_set_volt(uint32_t mode)
{
	uint32_t key =
		boot_main_cmu_frequency_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	uint16_t target_ldo, target_dcdc, unused;
	uint16_t previous_ldo, previous_low;

	if (mode == 0U) {
		mode = boot_pmu_power_mode;
	}
	pmu_dig_get_target_volt(&target_ldo, &target_dcdc, &unused);
	(void)pmu_module_get_volt(6U, &previous_low, &previous_ldo);
	uint16_t value = boot_power_read16(0x46U);
	uint16_t previous_dcdc = calibrated(value & 255U, boot_pmu_dcdc_dig_comp);
	uint16_t previous_dcdc_low = calibrated(value >> 8, boot_pmu_dig_lp_comp);

	if (mode == 3U) {
		if (target_dcdc > previous_dcdc) {
			do {
				previous_dcdc++;
				pmu_dcdc_dig_set_volt(previous_dcdc, boot_dig_lp_dcdc);
			} while (target_dcdc > previous_dcdc);
			pmu_module_set_volt(6U, boot_dig_lp_ldo, target_ldo);
			hal_sys_timer_delay_us(10U);
		} else {
			if (target_dcdc != previous_dcdc || previous_dcdc_low != boot_dig_lp_dcdc) {
				pmu_dcdc_dig_set_volt(target_dcdc, boot_dig_lp_dcdc);
			}
			pmu_module_set_volt(6U, boot_dig_lp_ldo, target_ldo);
		}
	} else {
		pmu_module_ramp_volt(6U, boot_dig_lp_ldo, target_ldo);
		pmu_dcdc_dig_set_volt(target_dcdc, boot_dig_lp_dcdc);
		if (previous_ldo < target_ldo) {
			hal_sys_timer_delay_us(10U);
		}
	}
	uint32_t threshold = boot_pmu_type == 2U ? 68U : 136U;
	uint32_t enabled = target_dcdc >= threshold;

	value = boot_power_read16(0x2028U);
	if (((value & 0x8000U) != 0U) != enabled) {
		boot_power_write16(0x2028U, enabled ? value | 0x8000U : value & 0x7fffU);
	}
	boot_main_cmu_frequency_irq_unlock(key);
}

NORMAL(pmu_mode_change)
void pmu_mode_change(uint32_t mode)
{
	uint32_t key =
		boot_main_cmu_frequency_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	uint32_t previous_mode = boot_pmu_power_mode;
	uint16_t target = boot_ana_act_dcdc;
	uint16_t value = 0U;

	if (previous_mode == 2U || previous_mode == 3U) {
		value = boot_power_read16(0x47U);
		uint16_t previous = calibrated(value & 255U, boot_pmu_dcdc_ana_comp);
		uint16_t previous_low = calibrated(value >> 8, boot_pmu_ana_lp_comp);

		if (target > previous) {
			do {
				previous++;
				pmu_dcdc_ana_set_volt(previous, boot_ana_lp_dcdc);
			} while (target > previous);
			hal_sys_timer_delay_us(10U);
		} else if (target != previous || previous_low != boot_ana_lp_dcdc) {
			pmu_dcdc_ana_set_volt(target, boot_ana_lp_dcdc);
		}
	} else {
		pmu_dcdc_ana_set_volt(target, boot_ana_lp_dcdc);
	}
	pmu_dig_set_volt(boot_pmu_power_mode);
	switch (mode) {
	case 2U:
		if (boot_pmu_power_mode == 3U) {
			pmu_module_config(6U, 0U, 1U, 1U, 1U);
			hal_sys_timer_delay_us(1800U);
			if (boot_pmu_type == 1U || boot_pmu_type == 2U) {
				(void)hal_sys_timer_delay(80U);
			}
			value = update(0x15U, 3U, 2U);
		} else {
			value = update(0x4aU, 0x303U, 0x203U);
			hal_sys_timer_delay_us(100U);
			pmu_module_config(0U, 1U, 0U, 1U, 0U);
		}
		break;
	case 3U:
		value = update(0x4aU, 0x303U, 0x203U);
		value = boot_power_read16(0x15U);
		if (boot_pmu_type == 1U || boot_pmu_type == 2U) {
			value &= 0x3fffU;
		}
		value = (value & 0xfcfcU) | 0x203U;
		boot_power_write16(0x15U, value);
		hal_sys_timer_delay_us(100U);
		pmu_module_config(0U, 1U, 0U, 1U, 0U);
		pmu_module_config(6U, 1U, 0U, 1U, 0U);
		break;
	case 1U:
		pmu_module_config(0U, 0U, 1U, 1U, 1U);
		pmu_module_config(6U, 0U, 1U, 1U, 1U);
		if (boot_pmu_type == 2U && boot_pmu_metal_id <= 1U) {
			value = update(0x120U, 0U, 0x1cU);
			value = update(0x0dU, 0U, 0xc000U);
		}
		hal_sys_timer_delay_us(1800U);
		if (boot_pmu_type == 1U || boot_pmu_type == 2U) {
			(void)hal_sys_timer_delay(80U);
			if (boot_pmu_type == 2U && boot_pmu_metal_id <= 1U) {
				boot_power_write16(0x0dU, value & 0x7fffU);
				value = update(0x120U, 0x1cU, 4U);
			}
		}
		value = update(0x15U, 3U, 2U);
		value = update(0x4aU, 3U, 2U);
		break;
	default:
		break;
	}
	boot_pmu_power_mode = (uint8_t)mode;
	boot_main_cmu_frequency_irq_unlock(key);
}

SRAM(pmu_sys_freq_config)
void pmu_sys_freq_config(uint32_t frequency)
{
	uint32_t key =
		boot_main_cmu_frequency_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	uint32_t previous = boot_pmu_vcore_req;
	uint32_t requested = previous & ~0x38U;

	if (frequency > 14U) {
		requested |= 0x10U;
	} else if (frequency == 14U) {
		requested |= 8U;
	}
	boot_pmu_vcore_req = requested;
	boot_main_cmu_frequency_irq_unlock(key);
	if (previous != requested) {
		if (boot_pmu_power_mode != 0U) {
			pmu_dig_set_volt(0U);
		} else {
			pmu_dig_init_volt();
		}
	}
}
