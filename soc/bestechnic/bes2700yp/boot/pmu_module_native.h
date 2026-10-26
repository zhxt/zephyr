/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_PMU_MODULE_NATIVE_H_
#define BOOT_PMU_MODULE_NATIVE_H_

#include <stdint.h>

#define BOOT_PMU_MODULE_COUNT 11U

struct boot_pmu_module_config {
	uint16_t manual;
	uint16_t enabled;
	uint16_t low_power_manual;
	uint16_t low_power_enabled;
	uint16_t sleep_enabled;
	uint16_t low_voltage_mask;
	uint16_t low_voltage_shift;
	uint16_t active_voltage_mask;
	uint16_t active_voltage_shift;
};

extern struct boot_pmu_module_config boot_pmu_module_cfg[BOOT_PMU_MODULE_COUNT];

void pmu_module_config(uint32_t module, uint32_t manual, uint32_t enabled, uint32_t low_power,
		       uint16_t sleep);
void pmu_module_set_volt(uint32_t module, uint32_t low, uint32_t active);
int pmu_module_get_volt(uint32_t module, uint16_t *low, uint16_t *active);
void pmu_module_ramp_volt(uint32_t module, uint32_t low, uint32_t active);

#endif /* BOOT_PMU_MODULE_NATIVE_H_ */
