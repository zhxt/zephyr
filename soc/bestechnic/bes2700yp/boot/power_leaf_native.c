/* SPDX-License-Identifier: Apache-2.0 */

#include "power_leaf_native.h"
#include "power_transfer.h"
#include "timer_native.h"

#define BOOT(name) __attribute__((section(".boot_text_flash." #name), noinline))
#define NORMAL(name) __attribute__((section(".text." #name), noinline))

__attribute__((section(".boot_bss.boot_power_vcm_pmu_only")))
uint8_t boot_power_vcm_pmu_only;

BOOT(rc_clock_init)
void rc_clock_init(void)
{
	boot_power_write16(0x4001U, boot_power_read16(0x4001U) | 0xe080U);
	boot_power_write16(0x4009U, boot_power_read16(0x4009U) & ~0x0780U);
	boot_power_write16(0x400aU, boot_power_read16(0x400aU) & ~0x6000U);
}

NORMAL(rc_vcm_pmu_only)
void rc_vcm_pmu_only(uint32_t value)
{
	boot_power_vcm_pmu_only = (uint8_t)value;
}

BOOT(xtal_clock_init)
void xtal_clock_init(void)
{
	boot_power_write16(0x3002U, 0x003fU);
	boot_power_write16(0x300eU, 0x0003U);
	boot_power_write16(0x3004U, boot_power_read16(0x3004U) | 0x0004U);
	boot_power_write16(0x301eU, 0U);
	boot_power_write16(0x301fU, 0U);
	boot_power_write16(0x3020U, 0U);
	boot_power_write16(0x3021U, 0x1f00U);
	boot_power_write16(0x3022U, 0xfffeU);
}

NORMAL(xtal_usb_bg_enable)
void xtal_usb_bg_enable(uint32_t enabled)
{
	uint16_t value = boot_power_read16(0x3001U);

	boot_power_write16(0x3001U, enabled != 0U ? value | 3U : value & ~3U);
}

BOOT(usbphy_ldo_config)
void usbphy_ldo_config(uint32_t enabled)
{
	uint16_t value = boot_power_read16(0x7011U);

	boot_power_write16(0x7011U, enabled != 0U ? value | 0x4040U :
								(value & ~0x4040U) | 0x0040U);
	if (enabled != 0U) {
		hal_sys_timer_delay(48U);
	}
}
