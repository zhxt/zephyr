/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_POWER_LEAF_NATIVE_H_
#define BOOT_POWER_LEAF_NATIVE_H_

#include <stdint.h>

/* Fixed serialized loader ABI; enable predicates inspect the full word. */
void rc_clock_init(void);
void rc_vcm_pmu_only(uint32_t value);
void xtal_clock_init(void);
void xtal_usb_bg_enable(uint32_t enabled);
void usbphy_ldo_config(uint32_t enabled);

extern uint8_t boot_power_vcm_pmu_only;

/* Failed transfers stop through the existing bounded loader diagnostics. */
void boot_power_leaf_fail(uint32_t operation, uint32_t address, int status)
	__attribute__((noreturn));

#endif /* BOOT_POWER_LEAF_NATIVE_H_ */
