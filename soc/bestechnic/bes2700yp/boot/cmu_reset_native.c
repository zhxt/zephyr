/* SPDX-License-Identifier: Apache-2.0 */

#include "cmu_reset_native.h"
#include "cmu_gate_native.h"
#include "psc_native.h"

struct reset_location {
	uintptr_t set;
	uintptr_t readback;
	uint32_t mask;
	uint32_t nops;
};

__attribute__((section(".boot_text_flash.boot_cmu_reset_location"), noinline))
static int reset_location(uint32_t module, struct reset_location *location)
{
	uint32_t bit;
	uintptr_t address;

	if (module > 185U) {
		return 0;
	}
	if (module <= 19U) {
		address = 0x40000034U;
		bit = module;
	} else if (module <= 29U) {
		address = 0x4000003cU;
		bit = module - 20U;
	} else if (module <= 40U) {
		address = 0x40000044U;
		bit = module - 30U;
	} else if (module <= 59U) {
		address = 0x40080010U;
		bit = module - 41U;
	} else if (module <= 70U) {
		address = 0x400800c8U;
		bit = module - 60U;
	} else if (module <= 90U) {
		address = 0x50000034U;
		bit = module - 71U;
	} else if (module <= 118U) {
		address = 0x5000003cU;
		bit = module - 91U;
	} else if (module <= 150U) {
		address = 0x50000044U;
		bit = module - 119U;
	} else if (module <= 167U) {
		address = 0x50000170U;
		bit = module - 151U;
	} else {
		address = 0x50000188U;
		bit = module - 168U;
	}
	location->set = address;
	location->mask = 1U << bit;
	location->nops = module <= 19U ? 2U : module <= 29U ? 4U : 0U;
	location->readback = module <= 40U ? 0U : module <= 70U ? 0x40080000U : 0x50000000U;
	return 1;
}

__attribute__((section(".boot_text_flash.hal_cmu_reset_set"), noinline))
int hal_cmu_reset_set(uint32_t module)
{
	struct reset_location location;

	if (!reset_location(module, &location)) {
		return 1;
	}
	if (module == 70U) {
		hal_psc_ram_ret_during_reset();
		hal_cmu_reset_osc_clock_sel();
	}
	boot_cmu_gate_write(location.set, location.mask);
	return 0;
}

__attribute__((section(".boot_text_flash.hal_cmu_reset_clear"), noinline))
int hal_cmu_reset_clear(uint32_t module)
{
	struct reset_location location;

	if (!reset_location(module, &location)) {
		return 1;
	}
	boot_cmu_gate_write(location.set + 4U, location.mask);
	if (location.nops != 0U) {
		boot_cmu_reset_nops(location.nops);
	}
	if (location.readback != 0U) {
		(void)boot_cmu_gate_read(location.readback);
	}
	return 0;
}
