/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>
#include <zephyr/arch/common/sys_bitops.h>

#include "bootmode_native.h"

/* Fixed-build experiment mappings, without a public BES register contract.
 * The masks describe observed bus values, not named reset/wakeup causes.
 */
#define BOOTMODE_SNAPSHOT_MASK 0x0fU
#define BOOTMODE_CONFIRM_MASK 0x0fU

#ifdef BOOTMODE_MMIO_MODEL
extern volatile uint32_t bootmode_model_register;
#define BOOTMODE_STATUS_ADDRESS ((uintptr_t)&bootmode_model_register)
#else
#define BOOTMODE_STATUS_ADDRESS ((uintptr_t)0x40080038U)
#endif

__attribute__((section(".boot_text_flash.boot_mode_hw_capture")))
uint8_t boot_mode_hw_capture(void)
{
	return (uint8_t)(sys_read32(BOOTMODE_STATUS_ADDRESS) & BOOTMODE_SNAPSHOT_MASK);
}

__attribute__((section(".boot_text_flash.boot_mode_hw_acknowledge")))
void boot_mode_hw_acknowledge(void)
{
	/* A fresh read preserves the value present at confirmation time. */
	sys_set_bits(BOOTMODE_STATUS_ADDRESS, BOOTMODE_CONFIRM_MASK);
}
