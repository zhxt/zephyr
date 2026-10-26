/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <zephyr/sys/util.h>

#include "pmu_module_native.h"
#include "pmu_voltage_native.h"
#include "power_transfer.h"

#define ENTRY(name) __attribute__((section(".text." #name), noinline))
#define INLINE      __attribute__((always_inline)) static inline

INLINE void valid_module(uint32_t module)
{
	if (module >= BOOT_PMU_MODULE_COUNT) {
		boot_power_leaf_fail(3U, module, -EINVAL);
	}
}

INLINE int absent(uint32_t type, uint32_t module)
{
	if (type == 1U || type == 2U) {
		return module == 1U || module == 4U || (module > 6U && module != 10U);
	}
	return (type == 0U || type == 3U) && module == 10U;
}

INLINE uint32_t module_register(uint32_t module)
{
	return module == 10U                  ? 0x115U
	       : module == 9U                 ? 0x43U
	       : module == 7U || module == 8U ? module + 8U
					      : module + 7U;
}

INLINE uint16_t flag(uint16_t value, uint16_t mask, uint32_t enabled)
{
	return enabled != 0U ? value | mask : value & (uint16_t)~mask;
}

INLINE uint16_t control(uint16_t value, const struct boot_pmu_module_config *config,
			uint32_t manual, uint32_t enabled)
{
	value = flag(value, config->manual, manual);
	return flag(value, config->enabled, enabled);
}

ENTRY(pmu_module_config)
void pmu_module_config(uint32_t module, uint32_t manual, uint32_t enabled, uint32_t low_power,
		       uint16_t sleep)
{
	valid_module(module);
	if (absent(boot_pmu_type, module)) {
		return;
	}
	const struct boot_pmu_module_config *config = &boot_pmu_module_cfg[module];
	uint32_t address = module_register(module);
	uint16_t value;

	if (module == 7U || module == 8U) {
		uint32_t sleep_address = module == 7U ? 0x44U : 0x0fU;

		value = boot_power_read16(sleep_address);
		boot_power_write16(sleep_address, flag(value, config->sleep_enabled, sleep));
		value = boot_power_read16(address);
		value = control(value, config, manual, enabled);
	} else if (module == 0U || module == 1U || module == 10U) {
		uint32_t control_address = module == 0U ? 3U : module == 1U ? 0x15U : 0x116U;

		value = boot_power_read16(control_address);
		boot_power_write16(control_address, control(value, config, manual, enabled));
		value = boot_power_read16(address);
	} else {
		value = boot_power_read16(address);
		value = control(value, config, manual, enabled);
	}
	if (low_power != 0U) {
		value &= (uint16_t)~config->low_power_manual;
	} else {
		value = (value & (uint16_t)~config->low_power_enabled) | config->low_power_manual;
	}
	if (module != 7U && module != 8U) {
		value = flag(value, config->sleep_enabled, sleep);
	}
	boot_power_write16(address, value);
}

INLINE uint32_t clip6(uint32_t code, int8_t adjustment)
{
	int32_t sum = (int32_t)(code + (uint32_t)(int32_t)adjustment);

	return (uint32_t)CLAMP(sum, 0, 63);
}

INLINE uint16_t insert(uint16_t value, uint32_t code, uint16_t mask, uint16_t shift)
{
	/* ARM register shifts use the low byte and produce zero at 32 or above. */
	uint32_t bits = (shift & 255U) < 32U ? code << (shift & 255U) : 0U;

	return (value & (uint16_t)~mask) | (bits & mask);
}

INLINE uint16_t extract(uint16_t value, uint16_t mask, uint16_t shift)
{
	return (shift & 255U) < 32U ? (uint32_t)(value & mask) >> (shift & 255U) : 0U;
}

INLINE void split_set(uint32_t module, uint32_t active)
{
	if (module == 2U) {
		uint16_t value = boot_power_read16(0xbbU);

		boot_power_write16(0xbbU, insert(value, active, 0x4000U, 14U));
		value = boot_power_read16(9U);
		uint32_t code = (active >> 1) & 31U;

		boot_power_write16(9U, (value & ~0x3ffU) | (code << 5) | code);
	} else {
		uint16_t value = boot_power_read16(0xbcU);

		boot_power_write16(0xbcU, insert(value, active, 0x6000U, 13U));
		value = boot_power_read16(12U);
		uint32_t type = boot_pmu_type;
		uint32_t code = (active >> 2) & 15U;
		uint32_t mask = type == 0U || type == 3U ? 0xffU : 0x3ffU;
		uint32_t shift = type == 0U || type == 3U ? 4U : 5U;

		boot_power_write16(12U, (value & ~mask) | (code << shift) | code);
	}
}

ENTRY(pmu_module_set_volt)
void pmu_module_set_volt(uint32_t module, uint32_t low, uint32_t active)
{
	valid_module(module);
	uint32_t type = boot_pmu_type;

	if (absent(type, module)) {
		return;
	}
	if (type == 3U && (module == 2U || module == 5U)) {
		split_set(module, active);
		return;
	}
	uint32_t transition = active;

	if (module == 6U || module == 10U) {
		int8_t a = module == 6U ? boot_pmu_ldo_dig_m_comp : boot_pmu_ldo_dig_l_comp;
		int8_t b = module == 6U ? boot_pmu_ldo_lp_m_comp : boot_pmu_ldo_lp_l_comp;

		active = clip6(active, a);
		low = clip6(low, b);
		if (type == 1U || type == 2U) {
			transition = clip6(transition, b);
		}
	}
	uint32_t address = module_register(module);
	uint16_t value = boot_power_read16(address);
	const struct boot_pmu_module_config *config = &boot_pmu_module_cfg[module];

	value = insert(value, active, config->active_voltage_mask, config->active_voltage_shift);
	value = insert(value, low, config->low_voltage_mask, config->low_voltage_shift);
	boot_power_write16(address, value);
	type = boot_pmu_type;
	if (type == 1U || type == 2U) {
		if (module == 6U || module == 10U || module == 3U || module == 5U) {
			address = module == 6U || module == 10U ? 0x72U : 0x73U;
			uint32_t shift = module == 6U ? 6U : module == 5U ? 5U : 0U;
			uint16_t mask = module == 6U    ? 0xfc0U
					: module == 10U ? 0x3fU
					: module == 5U  ? 0x3e0U
							: 0x1fU;

			value = boot_power_read16(address);
			boot_power_write16(address, insert(value, transition, mask, shift));
		}
	}
}

ENTRY(pmu_module_get_volt)
int pmu_module_get_volt(uint32_t module, uint16_t *low, uint16_t *active)
{
	if (module >= BOOT_PMU_MODULE_COUNT) {
		return -EINVAL;
	}
	uint32_t type = boot_pmu_type;

	if (absent(type, module)) {
		if (active != NULL) {
			*active = 0U;
		}
		if (low != NULL) {
			*low = 0U;
		}
		return 0;
	}
	if (type == 3U && (module == 2U || module == 5U)) {
		uint16_t high = boot_power_read16(module == 2U ? 0xbbU : 0xbcU);
		uint16_t value = boot_power_read16(module == 2U ? 9U : 12U);
		uint32_t code;

		if (module == 2U) {
			code = ((high >> 14) & 1U) | (((value >> 5) & 31U) << 1);
		} else {
			type = boot_pmu_type;
			uint32_t shift = type == 0U || type == 3U ? 4U : 5U;
			uint32_t mask = type == 0U || type == 3U ? 15U : 31U;

			code = ((high >> 13) & 3U) | (((value >> shift) & mask) << 2);
		}
		if (active != NULL) {
			*active = (uint16_t)code;
		}
		if (low != NULL) {
			*low = (uint16_t)code;
		}
		return 0;
	}
	uint16_t value = boot_power_read16(module_register(module));
	const struct boot_pmu_module_config *config = &boot_pmu_module_cfg[module];

	if (active != NULL) {
		*active = extract(value, config->active_voltage_mask, config->active_voltage_shift);
	}
	if (low != NULL) {
		*low = extract(value, config->low_voltage_mask, config->low_voltage_shift);
	}
	if (module == 6U || module == 10U) {
		if (active != NULL) {
			int8_t comp =
				module == 6U ? boot_pmu_ldo_dig_m_comp : boot_pmu_ldo_dig_l_comp;

			*active = (uint16_t)CLAMP((int32_t)*active - comp, 0, 63);
		}
		if (low != NULL) {
			int8_t comp =
				module == 6U ? boot_pmu_ldo_lp_m_comp : boot_pmu_ldo_lp_l_comp;

			*low = (uint16_t)CLAMP((int32_t)*low - comp, 0, 63);
		}
	}
	return 0;
}

ENTRY(pmu_module_ramp_volt)
void pmu_module_ramp_volt(uint32_t module, uint32_t low, uint32_t active)
{
	valid_module(module);
	uint16_t previous_low = 0U, previous_active = 0U;

	(void)pmu_module_get_volt(module, &previous_low, &previous_active);
	if (previous_active < active) {
		do {
			uint32_t next =
				(uint32_t)MIN((int32_t)previous_active + 2, (int32_t)active);

			previous_active = (uint16_t)next;
			pmu_module_set_volt(module, low, next);
		} while (previous_active < active);
	} else if (previous_active != active || previous_low != low) {
		pmu_module_set_volt(module, low, active);
	}
}
