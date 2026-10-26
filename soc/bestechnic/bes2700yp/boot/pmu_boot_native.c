/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <zephyr/sys/util.h>

#include "bootmode_native.h"
#include "main_cmu_leaf_native.h"
#include "pmu_boot_native.h"
#include "pmu_peripheral_native.h"
#include "pmu_voltage_native.h"
#include "itn_native.h"
#include "power_transfer.h"
#include "timer_native.h"

__attribute__((section(".boot_text_flash.pmu_boot_init"), noinline)) void pmu_boot_init(void)
{
	uint16_t initial = 0U;
	uint32_t bootmode = hal_hw_bootmode_get().reg;

	if (boot_pmu_type == 2U) {
		initial = boot_power_read16(0x11U);
	}
	if ((bootmode & 1U) != 0U) {
		boot_pmu_boot_reason = 2U;
	} else if ((bootmode & 2U) != 0U) {
		boot_pmu_boot_reason = 4U;
	} else if (boot_pmu_type == 2U) {
		uint16_t watchdog = boot_power_read16(0x10cU);
		uint16_t power = boot_power_read16(0x52U);
		uint16_t cause = boot_power_read16(0x5eU);

		if ((initial & 8U) != 0U) {
			boot_pmu_boot_reason = 0x100U;
		} else if ((initial & 1U) != 0U) {
			boot_pmu_boot_reason = (watchdog & 0x180U) != 0U ? 0x40U : 0x80U;
		} else if ((power & 0x2000U) != 0U) {
			boot_pmu_boot_reason = 0x20U;
		} else {
			boot_pmu_boot_reason = (cause & 4U) != 0U ? 8U : 1U;
		}
	}
	if (boot_pmu_type == 2U) {
		boot_pmu_boot_first_pwr_up = (uint8_t)((initial >> 2) & 1U);
		boot_power_write16(0x11U, (initial & (uint16_t)~0xcU) | 1U);
	}
	/* The retained ABI publishes this global output inside the read callback. */
	int status = hal_analogif_reg_read(0x5eU, &boot_pmu_chg_ctx);

	if (status != 0) {
		boot_power_leaf_fail(0U, 0x5eU, status);
	}
	boot_power_write16(0U, 0xcafeU);
	boot_power_write16(0U, 0x5feeU);
	hal_sys_timer_delay(9U);
	if (boot_pmu_type == 2U) {
		static const uint16_t clear[] = {0x12aU, 0x12fU, 0x132U, 0x136U,
						 0x139U, 0x13dU, 0x140U};

		for (uint32_t i = 0U; i < ARRAY_SIZE(clear); i++) {
			boot_power_write16(clear[i], 0xffU);
		}
		boot_power_write16(0x144U, 0xff0U);
		boot_power_write16(0x129U, 0U);
	}
	pmu_wdt_stop();
	pmu_itn_init();
	uint32_t type = boot_pmu_type;

	if (type == 0U || type == 3U) {
		hal_cmu_boot_dcdc_clock_enable();
		for (uint32_t code = 0U; code < 0xc000U; code += 0x2000U) {
			boot_power_write16(0x3fU, 0x1100U);
			boot_power_write16(0x40U, (uint16_t)(code | 0x828U));
			uint16_t value = boot_power_read16(0x4fU);

			boot_power_write16(0x4fU, (value & 0xfffU) | 0xa000U);
			hal_sys_timer_delay(9U);
			uint32_t start = hal_sys_timer_get();

			do {
				value = boot_power_read16(0x4fU);
				if ((uint32_t)(hal_sys_timer_get() - start) > 800U) {
					boot_power_leaf_fail(4U, 0x4fU, -ETIMEDOUT);
				}
			} while ((value & 0xc0U) != 0x40U);
			boot_power_write16(0x3fU, 0x1000U);
		}
		hal_cmu_boot_dcdc_clock_disable();
	}
	pmu_rf_ana_init();
}
