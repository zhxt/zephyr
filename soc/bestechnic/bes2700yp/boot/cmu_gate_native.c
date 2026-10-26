/* SPDX-License-Identifier: Apache-2.0 */

#include "cmu_gate_native.h"

struct gate_location {
	uintptr_t enable;
	uintptr_t status;
	uint32_t mask;
};

__attribute__((section(".boot_text_flash.boot_cmu_gate_location"), noinline))
static int gate_location(uint32_t module, struct gate_location *location)
{
	uint32_t bit;
	uintptr_t address;

	if (module > 185U || (module >= 60U && module <= 70U)) {
		return 0;
	}
	if (module <= 19U) {
		address = 0x40000000U;
		bit = module;
	} else if (module <= 29U) {
		address = 0x40000008U;
		bit = module - 20U;
	} else if (module <= 40U) {
		address = 0x40000010U;
		bit = module - 30U;
	} else if (module <= 59U) {
		address = 0x40080040U;
		bit = module - 41U;
	} else if (module <= 90U) {
		address = 0x50000000U;
		bit = module - 71U;
	} else if (module <= 118U) {
		address = 0x50000008U;
		bit = module - 91U;
	} else if (module <= 150U) {
		address = 0x50000010U;
		bit = module - 119U;
	} else if (module <= 167U) {
		address = 0x50000160U;
		bit = module - 151U;
	} else {
		address = 0x50000178U;
		bit = module - 168U;
	}
	location->enable = address;
	location->status = address + (module >= 71U ? 4U : 0U);
	location->mask = 1U << bit;
	return 1;
}

__attribute__((section(".boot_text_flash.hal_cmu_clock_enable"), noinline))
int hal_cmu_clock_enable(uint32_t module)
{
	struct gate_location location;

	if (!gate_location(module, &location)) {
		return 1;
	}
	boot_cmu_gate_write(location.enable, location.mask);
	if (module >= 41U && module <= 59U) {
		(void)boot_cmu_gate_read(0x40080000U);
	}
	return 0;
}

__attribute__((section(".boot_text_flash.hal_cmu_clock_disable"), noinline))
int hal_cmu_clock_disable(uint32_t module)
{
	struct gate_location location;

	if (!gate_location(module, &location)) {
		return 1;
	}
	boot_cmu_gate_write(location.enable + 4U, location.mask);
	return 0;
}

__attribute__((section(".boot_text_flash.hal_cmu_clock_get_status"), noinline))
int hal_cmu_clock_get_status(uint32_t module)
{
	struct gate_location location;

	if (!gate_location(module, &location)) {
		return 0;
	}
	return (boot_cmu_gate_read(location.status) & location.mask) != 0U;
}
