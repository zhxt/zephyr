/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include "itn_native.h"
#include "power_transfer.h"
#include "timer_native.h"

#define SRAM(name) __attribute__((section(".boot_text_sram." #name), noinline))
#define FLASH(name) __attribute__((section(".boot_text_flash." #name), noinline))
#define NORMAL(name) __attribute__((section(".text." #name), noinline))

__attribute__((section(".boot_bss.boot_itn_efuse_valid"))) uint8_t boot_itn_efuse_valid;
uint16_t boot_itn_efuse_cache[32];

SRAM(pmu_get_efuse_phy)
int pmu_get_efuse_phy(uint32_t page, uint16_t *value)
{
	uint32_t selection = (page << 6) | 0x18U;
	uint32_t clear = 8U, final = 0U;
	uint16_t lower = 0U, upper = 0U;
	int status, cleanup_status;

	status = hal_analogif_reg_write(0x8046U, 8U);
	if (status != 0) {
		goto cleanup;
	}
	status = hal_analogif_reg_write(0x8046U, 0x18U);
	if (status != 0) {
		goto cleanup;
	}
	status = hal_analogif_reg_write(0x8046U, selection);
	if (status != 0) {
		goto cleanup;
	}
	status = hal_analogif_reg_write(0x8046U, selection | 0x20U);
	if (status != 0) {
		clear = 0x28U;
		final = 0x20U;
		goto cleanup;
	}
	status = hal_analogif_reg_write(0x8046U, selection);
	if (status != 0) {
		goto cleanup;
	}
	status = hal_analogif_reg_read(0x804dU, &lower);
	if (status != 0) {
		goto cleanup;
	}
	status = hal_analogif_reg_read(0x804cU, &upper);
	if (status != 0) {
		goto cleanup;
	}
	*value = lower | upper;
cleanup:
	cleanup_status = hal_analogif_reg_write(0x8046U, clear);
	if (status == 0) {
		status = cleanup_status;
	}
	cleanup_status = hal_analogif_reg_write(0x8046U, final);
	return status != 0 ? status : cleanup_status;
}

SRAM(pmu_efuse_init)
void pmu_efuse_init(void)
{
	boot_itn_efuse_valid = 0U;
	for (uint32_t page = 0U; page < 32U; page++) {
		int status = pmu_get_efuse_phy(page, &boot_itn_efuse_cache[page]);

		if (status != 0) {
			boot_power_leaf_fail(2U, page, status);
		}
	}
	boot_itn_efuse_valid = 1U;
}

SRAM(pmu_get_efuse)
int pmu_get_efuse(uint32_t page, uint16_t *value)
{
	if (boot_itn_efuse_valid == 0U) {
		return pmu_get_efuse_phy(page, value);
	}
	if (page >= 32U) {
		return -EINVAL;
	}
	*value = boot_itn_efuse_cache[page];
	return 0;
}

FLASH(pmu_itn_init)
void pmu_itn_init(void)
{
	boot_power_write16(0x8000U, 0xcafeU);
	boot_power_write16(0x8000U, 0x5feeU);
	hal_sys_timer_delay(9U);
	boot_power_write16(0x801bU, 0U);
	boot_power_write16(0x8002U, boot_power_read16(0x8002U) & ~0x1000U);
	boot_power_write16(0x800aU, (boot_power_read16(0x800aU) & ~0x0c00U) | 0x0800U);
	hal_sys_timer_delay(16U);
	boot_power_write16(0x800bU, (boot_power_read16(0x800bU) & ~0x0c30U) | 0x0820U);
	hal_sys_timer_delay(16U);
	boot_power_write16(0x8018U, (boot_power_read16(0x8018U) & ~0x0c00U) | 0x0800U);
	uint32_t chip = pmu_get_chip_type();

	boot_power_write16(0x800dU, boot_power_read16(0x800dU) | 1U);
	hal_sys_timer_delay(16U);
	if ((uint8_t)(chip - 1U) <= 1U) {
		boot_power_write16(0x8008U, 0x0800U);
		boot_power_write16(0x8003U, 0xaaaaU);
	} else {
		boot_power_write16(0x8003U, (boot_power_read16(0x8003U) & ~0x0300U) | 0x0100U);
		hal_sys_timer_delay(16U);
	}
}

NORMAL(pmu_itn_close_usb_itf)
void pmu_itn_close_usb_itf(void)
{
	boot_power_write16(0x800aU, (boot_power_read16(0x800aU) & ~0x000cU) | 0x0008U);
	boot_power_write16(0x800eU, (boot_power_read16(0x800eU) & ~0x2100U) | 0x0100U);
}

NORMAL(pmu_itn_open_usb_itf)
void pmu_itn_open_usb_itf(void)
{
	uint16_t config = (boot_power_read16(0x800aU) & ~0x000cU) | 0x0008U;

	boot_power_write16(0x800aU, config);
	boot_power_write16(0x800eU, (boot_power_read16(0x800eU) & ~0x2100U) | 0x2000U);
	boot_power_write16(0x800aU, (config & ~0x000cU) | 0x0004U);
}
