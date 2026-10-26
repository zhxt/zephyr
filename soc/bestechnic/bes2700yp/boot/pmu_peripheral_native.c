/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/sys/util.h>

#include "itn_native.h"
#include "main_cmu_frequency_native.h"
#include "pmu_module_native.h"
#include "pmu_peripheral_native.h"
#include "pmu_sequence_native.h"
#include "pmu_voltage_native.h"
#include "power_transfer.h"
#include "timer_native.h"

#define SRAM(name)   __attribute__((section(".boot_text_sram." #name), noinline))
#define FLASH(name)  __attribute__((section(".boot_text_flash." #name), noinline))
#define NORMAL(name) __attribute__((section(".text." #name), noinline))

SRAM(pmu_wdt_stop)
void pmu_wdt_stop(void)
{
	uint32_t type = boot_pmu_type;
	uint32_t address = type == 0U || type == 3U ? 0x6bU : 0x10cU;
	uint16_t value = boot_power_read16(address);

	type = boot_pmu_type;
	address = type == 0U || type == 3U ? 0x6bU : 0x10cU;
	boot_power_write16(address, value & (uint16_t)~0x180U);
}

FLASH(pmu_rf_ana_init)
void pmu_rf_ana_init(void)
{
	static const uint16_t banks[] = {0x3000U, 0x2000U, 0x1000U, 0x4000U};
	static const struct {
		uint16_t address;
		uint16_t value;
	} settings[] = {
		{0x20c4U, 0x9f94U}, {0x20c5U, 0x0207U}, {0x20ecU, 0xdd2fU}, {0x20edU, 0x0624U},
		{0x20eeU, 0x0001U}, {0x20f0U, 0x0001U}, {0x20aeU, 0x26a5U}, {0x233aU, 0x0707U},
		{0x2340U, 0x3908U}, {0x2341U, 0xffffU},
	};

	for (uint32_t i = 0U; i < ARRAY_SIZE(banks); i++) {
		boot_power_write16(banks[i], 0xcafeU);
		boot_power_write16(banks[i], 0x5feeU);
		hal_sys_timer_delay(9U);
	}
	for (uint32_t i = 0U; i < ARRAY_SIZE(settings); i++) {
		boot_power_write16(settings[i].address, settings[i].value);
	}
	uint32_t type = boot_pmu_type;

	if (type == 1U || type == 2U) {
		if (boot_pmu_metal_id != 0U) {
			uint16_t value = boot_power_read16(0x102U);

			boot_power_write16(0x102U, value & (uint16_t)~0x200U);
		}
		type = boot_pmu_type;
		boot_power_write16(0x6cU,
				   type == 1U && boot_pmu_metal_id == 0U ? 0x8777U : 0x0777U);
	}
}

NORMAL(pmu_usb_config)
void pmu_usb_config(uint32_t enabled)
{
	uint32_t caller = (uint32_t)(uintptr_t)__builtin_return_address(0);
	uint32_t on = enabled != 0U;

	if (on != 0U) {
		pmu_itn_open_usb_itf();
		usbphy_ldo_config(1U);
	} else {
		usbphy_ldo_config(0U);
		pmu_itn_close_usb_itf();
	}
	uint32_t key = boot_main_cmu_frequency_irq_lock(caller);

	xtal_usb_bg_enable(on);
	uint32_t type = boot_pmu_type;

	if (type == 1U || type == 2U) {
		uint16_t value = boot_power_read16(0x63U);

		boot_power_write16(0x63U, on != 0U ? value | 0xc000U : value & 0x3fffU);
		value = boot_power_read16(0x6bU);
		boot_power_write16(0x6bU, on != 0U ? value | 0xc000U : (value & 0x3fffU) | 0x4000U);
	}
	boot_main_cmu_frequency_irq_unlock(key);
	if (on != 0U) {
		(void)osDelay(600U);
	}
	key = boot_main_cmu_frequency_irq_lock(caller);
	pmu_module_config(5U, 1U, on, 1U, (uint16_t)on);
	boot_main_cmu_frequency_irq_unlock(key);
	if (boot_pmu_power_mode != 0U) {
		pmu_dig_set_volt(0U);
	}
}
