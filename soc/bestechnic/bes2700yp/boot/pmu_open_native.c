/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/sys/util.h>

#include "boot_diagnostics.h"
#include "itn_native.h"
#include "pmu_module_native.h"
#include "pmu_open_native.h"
#include "pmu_peripheral_native.h"
#include "pmu_sequence_native.h"
#include "pmu_voltage_native.h"
#include "power_transfer.h"
#include "timer_native.h"

static void update(uint32_t address, uint16_t clear, uint16_t set)
{
	uint16_t value = boot_power_read16(address);

	boot_power_write16(address, (value & (uint16_t)~clear) | set);
}

static void read_then_write(uint32_t address, uint16_t value)
{
	(void)boot_power_read16(address);
	boot_power_write16(address, value);
}

static uint16_t efuse(uint32_t page)
{
	uint16_t value = 0U;
	int status = pmu_get_efuse(page, &value);

	if (status != 0) {
		boot_power_leaf_fail(2U, page, status);
	}
	return value;
}

static int8_t signed_magnitude(uint16_t word, uint32_t shift, uint32_t bits)
{
	int magnitude = (word >> shift) & ((1U << bits) - 1U);

	return (int8_t)((word & (1U << (shift + bits))) != 0U ? -magnitude : magnitude);
}

static void descriptors(void)
{
	static const struct boot_pmu_module_config initial[BOOT_PMU_MODULE_COUNT] = {
		{8U, 4U, 0x1000U, 0x800U, 0x400U, 0x3e0U, 5U, 0x1fU, 0U},
		{0x2000U, 0x1000U, 0x2000U, 0x1000U, 0U, 0xfc0U, 6U, 0x3fU, 0U},
		{0x800U, 0x400U, 0x4000U, 0x2000U, 0x1000U, 0x1fU, 0U, 0x3e0U, 5U},
		{0x800U, 0x400U, 0x4000U, 0x2000U, 0x1000U, 0x1fU, 0U, 0x3e0U, 5U},
		{0x800U, 0x400U, 0x4000U, 0x2000U, 0x1000U, 0x1fU, 0U, 0x3e0U, 5U},
		{0x800U, 0x400U, 0x4000U, 0x2000U, 0x1000U, 0x1fU, 0U, 0x3e0U, 5U},
		{0x2000U, 0x1000U, 0x8000U, 0x4000U, 0U, 0xfc0U, 6U, 0x3fU, 0U},
		{0x2000U, 0x1000U, 0x800U, 0x400U, 0x40U, 0x3e0U, 5U, 0x1fU, 0U},
		{0x8000U, 0x4000U, 0x1000U, 0x800U, 0x8000U, 0x1fU, 0U, 0x3e0U, 5U},
		{0x8000U, 0x4000U, 0x2000U, 0x1000U, 0U, 0xfc0U, 6U, 0x3fU, 0U},
		{0x2000U, 0x1000U, 0x2000U, 0x1000U, 0x8000U, 0xfc0U, 6U, 0x3fU, 0U},
	};
	uint32_t type = boot_pmu_type;

	for (uint32_t i = 0U; i < ARRAY_SIZE(initial); i++) {
		boot_pmu_module_cfg[i] = initial[i];
	}
	if (type == 0U) {
		boot_pmu_module_cfg[5] = (struct boot_pmu_module_config){
			0x200U, 0x100U, 0x1000U, 0x800U, 0x400U, 0xfU, 0U, 0xf0U, 4U};
	} else if (type == 3U) {
		boot_pmu_module_cfg[5].low_voltage_mask = 0xfU;
		boot_pmu_module_cfg[5].active_voltage_mask = 0xf0U;
		boot_pmu_module_cfg[5].active_voltage_shift = 4U;
	}
	boot_ana_act_dcdc = type == 2U ? 129U : type == 1U ? 80U : 84U;
	boot_ana_lp_dcdc = boot_ana_act_dcdc;
	boot_dig_lp_ldo = type == 1U ? 14U : 12U;
	boot_dig_lp_dcdc = type == 2U ? 45U : 88U;
}

static void type_two(void)
{
	(void)hal_trace_printf(0x42U, "PMU_CHIP_TYPE_1806, pmu_metal_id:%d", boot_pmu_metal_id);
	update(0x11dU, 0U, 1U);
	read_then_write(0x16U, 0x8e1fU);
	read_then_write(0x18U, 0xb424U);
	read_then_write(0x1aU, 0x8e1fU);
	read_then_write(0x1cU, 0xb444U);
	update(0x11cU, 0U, 1U);
	read_then_write(0x33U, 0x8e1fU);
	read_then_write(0x35U, 0xb424U);
	uint16_t value = boot_power_read16(0x39U);
	uint32_t type = boot_pmu_type;

	boot_power_write16(0x39U, value | (type == 0U || type == 3U ? 0xeU : 0x70U));
	read_then_write(0x41U, 0x1014U);
	value = boot_power_read16(0x42U);
	boot_power_write16(0x42U, (value & 0xffU) | 0x1c00U);
	update(0x11aU, 0U, 2U);
	boot_power_write16(0x116U, 0xe955U);
	if (boot_pmu_type == 2U) {
		update(0x61U, 0x3fe0U, 0xe0U);
	}
}

static void type_one(void)
{
	(void)hal_trace_printf(0x42U, "PMU_CHIP_TYPE_1805, pmu_metal_id:%d", boot_pmu_metal_id);
	uint32_t metal = boot_pmu_metal_id;

	if (metal == 1U) {
		update(0x102U, 0x6fU, 0x48U);
	} else if (metal > 1U) {
		update(4U, 0x600U, 0U);
		update(0x102U, 0x4221U, 0xa540U);
	}
	read_then_write(0x16U, 0x8e18U);
	read_then_write(0x1aU, 0x8e12U);
	read_then_write(0x33U, 0x8e18U);
	update(0x36U, 0U, 0x8000U);
	update(0x19U, 0U, 0x8000U);
	update(0x1dU, 0U, 0x8000U);
	if (boot_pmu_metal_id <= 1U) {
		update(0x11dU, 0U, 0x100U);
	}
	read_then_write(0x116U, 0xe955U);
	update(0x4cU, 0x0fffU, 0x410U);
	if (boot_pmu_metal_id == 0U) {
		update(0x6cU, 0x6000U, 0x8000U);
		hal_sys_timer_delay(80U);
		update(0x10U, 0x3ffU, 0x35aU);
		update(0x66U, 0U, 0x3000U);
		update(0x66U, 0x40U, 0U);
		update(0x6cU, 0U, 0x4000U);
	}
}

static void type_zero_or_three(void)
{
	static const struct {
		uint16_t address, value;
	} settings[] = {
		{0x16U, 0x8e1fU}, {0x17U, 0x211U}, {0x18U, 0xa444U}, {0x19U, 0xe210U},
		{0x1aU, 0x8e1fU}, {0x1bU, 0x211U}, {0x1cU, 0xa484U}, {0x1dU, 0xe210U},
		{0x33U, 0x8e1fU}, {0x34U, 0x211U}, {0x35U, 0xa484U}, {0x36U, 0xe210U},
	};
	(void)hal_trace_printf(0x42U,
			       boot_pmu_type == 0U ? "PMU_CHIP_TYPE_1803, pmu_metal_id:%d"
						   : "PMU_CHIP_TYPE_1808, pmu_metal_id:%d",
			       boot_pmu_metal_id);
	for (uint32_t i = 0U; i < ARRAY_SIZE(settings); i++) {
		boot_power_write16(settings[i].address, settings[i].value);
	}
	if (boot_pmu_type == 3U) {
		boot_power_write16(0x19U, 0xe230U);
		boot_power_write16(0x1dU, 0xe230U);
		boot_power_write16(0x36U, 0xe230U);
	}
	update(0x1bU, 0xf00U, 0x800U);
	update(0x17U, 0xf00U, 0x800U);
	update(0x34U, 0xf00U, 0x800U);
	uint16_t value = boot_power_read16(0x39U);
	uint32_t type = boot_pmu_type;

	boot_power_write16(0x39U, value | (type == 0U || type == 3U ? 8U : 0x40U));
	value = boot_power_read16(0x41U);
	boot_power_write16(0x41U, (value & 0xffU) | 0xe000U);
}

static void configuration(void)
{
	boot_power_write16(0x26U, 0U);
	boot_power_write16(0x27U, 0U);
	uint16_t value = boot_power_read16(0x5eU);

	boot_power_write16(0x5eU, value);
	value = boot_power_read16(0x52U);
	uint32_t type = boot_pmu_type;

	boot_power_write16(type == 0U || type == 3U ? 0x51U : 0x52U, value);
	update(2U, 0x1030U, 0x20U);
	if (boot_pmu_type == 3U) {
		update(0xbbU, 0U, 0x4000U);
		update(0xbcU, 0U, 0x6000U);
	}
	update(3U, 0x200U, 0U);
	type = boot_pmu_type;
	if (type == 2U || type == 1U) {
		uint16_t code = type == 2U ? 0x2000U : 0x3000U;

		update(0x1bU, 0x3f00U, code);
		update(0x17U, 0x3f00U, code);
		update(0x34U, 0x3f00U, code);
		if (type == 1U) {
			update(0x1cU, 0x1000U, 0U);
			update(0x18U, 0x1000U, 0U);
			update(0x35U, 0x1000U, 0U);
		}
	}
	type = boot_pmu_type;
	if (type == 2U) {
		type_two();
	} else if (type == 1U) {
		type_one();
	} else if (type == 0U || type == 3U) {
		type_zero_or_three();
	}
	type = boot_pmu_type;
	if (type == 1U || type == 2U) {
		update(0x63U, 0U, 0x3300U);
	}
}

static void calibration(void)
{
	uint16_t word = efuse(7U);

	boot_pmu_dcdc_dig_comp = signed_magnitude(word, 0U, 5U);
	boot_pmu_dcdc_ana_comp = signed_magnitude(word, 6U, 4U);
	boot_pmu_dcdc_hppa_comp = signed_magnitude(word, 11U, 4U);
	word = efuse(9U);
	boot_pmu_dig_lp_comp = signed_magnitude(word, 0U, 5U);
	boot_pmu_ana_lp_comp = signed_magnitude(word, 6U, 4U);
	boot_pmu_hppa_lp_comp = signed_magnitude(word, 11U, 4U);
	uint32_t type = boot_pmu_type;

	if (type == 0U || type == 3U) {
		boot_pmu_hppa_lp_comp = (int8_t)(boot_pmu_hppa_lp_comp + 3);
	}
	(void)hal_trace_printf(0x42U, "%s", "pmu_get_dcdc_calib_value");
	(void)hal_trace_printf(0x44U, "normal VCORE-M=%d VANA=%d VCODEC=%d", boot_pmu_dcdc_dig_comp,
			       boot_pmu_dcdc_ana_comp, boot_pmu_dcdc_hppa_comp);
	(void)hal_trace_printf(0x44U, "lowpwr VCORE-M=%d VANA=%d VCODEC=%d", boot_pmu_dig_lp_comp,
			       boot_pmu_ana_lp_comp, boot_pmu_hppa_lp_comp);
	pmu_get_ldo_dig_calib_value();
	type = boot_pmu_type;
	if (type == 1U) {
		update(0xdU, 0U, 0xc000U);
	}
	if (type == 1U || type == 2U) {
		update(0x15U, 0x3cU, 0U);
		if (type == 2U) {
			update(0x4aU, 0x3cU, 0U);
		}
		update(0x74U, 0xc00U, 0U);
		if (type == 1U) {
			update(0xdU, 0x8000U, 0U);
		} else {
			update(0x75U, 0xc00U, 0U);
		}
	}
}

static void disable_module(uint32_t module)
{
	pmu_module_config(module, 1U, 0U, 1U, 0U);
}

static void voltage(void)
{
	disable_module(1U);
	disable_module(9U);
	uint32_t type = boot_pmu_type;

	if (type != 1U && type != 2U) {
		update(0xeU, 0x6000U, 0U);
	}
	pmu_dcdc_hppa_set_volt(0xbcU, 0xbcU);
	update(0x4bU, 0x303U, 0x203U);
	disable_module(3U);
	disable_module(4U);
	if (boot_pmu_type == 1U) {
		update(0x10fU, 0x400U, 0x800U);
	}
	pmu_usb_config(0U);
	disable_module(8U);
	disable_module(7U);
	pmu_module_set_volt(8U, 5U, 5U);
	uint16_t value = boot_power_read16(0x48U);
	uint16_t active = (uint16_t)CLAMP((int)(value & 0xffU) - boot_pmu_dcdc_hppa_comp, 0, 255);
	int low_comp = boot_pmu_hppa_lp_comp;

	if (active < 0xacU) {
		for (uint32_t code = active + 1U; code <= 0xacU; code++) {
			pmu_dcdc_hppa_set_volt(code, 0xacU);
		}
	} else if (active != 0xacU || CLAMP((int)(value >> 8) - low_comp, 0, 255) != 0xac) {
		pmu_dcdc_hppa_set_volt(0xacU, 0xacU);
	}
	pmu_codec_mic_bias_set_volt(0x1fffffU, 2200U);
	if (boot_pmu_power_mode != 3U) {
		pmu_mode_change(3U);
	}
}

static void clock_and_reference(void)
{
	uint32_t type = boot_pmu_type;

	if (type == 1U || type == 2U) {
		update(0x11aU, 0U, 6U);
		update(0x120U, 0U, 0x400U);
		update(0x69U, 0U, 0xc0U);
	}
	type = boot_pmu_type;
	if (type == 1U || type == 2U) {
		uint16_t word = efuse(13U);

		if ((word & 0x1000U) != 0U) {
			rc_vcm_pmu_only(1U);
		}
	}
	type = boot_pmu_type;
	if (type == 1U || type == 2U) {
		update(0x1eU, 0U, 4U);
	}
	uint16_t word = efuse(14U);

	(void)hal_trace_printf(0x43U, "%s, efuse E : 0x%x", "pmu_set_sar_adc_vref", word);
	type = boot_pmu_type;
	uint16_t code = word & 0x3fU;

	if (code != 0U && type == 1U) {
		update(0x43U, 0xfc0U, (uint16_t)(code << 6));
	} else if (code != 0U && type == 2U) {
		update(0x25U, 0xfU, code & 0xfU);
	}
	if (boot_pmu_type == 1U) {
		update(0x1fU, 0U, 0x307U);
		update(0x25U, 0U, 0x2000U);
	}
	xtal_clock_init();
	rc_clock_init();
}

__attribute__((section(".boot_text_flash.pmu_open"), noinline)) int pmu_open(void)
{
	(void)hal_trace_printf(0x43U, "vhppa_mv:%d vcodec_mv:%d", 1700, 1700);
	pmu_efuse_init();
	descriptors();
	configuration();
	calibration();
	voltage();
	clock_and_reference();
	return 0;
}
