/* SPDX-License-Identifier: Apache-2.0 */

#include "cache_native.h"

__attribute__((section(".boot_text_sram.hal_cache_wrap_enable")))
uint8_t hal_cache_wrap_enable(uint32_t selector)
{
	if (selector == 0U) {
		uint32_t control = boot_cache_read32(BOOT_CACHE_CONTROL);

		boot_cache_write32(BOOT_CACHE_CONTROL, control | BOOT_CACHE_WRAP);
	}
	return 0U;
}

__attribute__((section(".boot_text_sram.hal_cache_wrap_disable")))
uint8_t hal_cache_wrap_disable(uint32_t selector)
{
	if (selector == 0U) {
		uint32_t control = boot_cache_read32(BOOT_CACHE_CONTROL);

		boot_cache_write32(BOOT_CACHE_CONTROL, control & ~BOOT_CACHE_WRAP);
	}
	return 0U;
}

__attribute__((section(".boot_text_sram.hal_cache_disable")))
uint8_t hal_cache_disable(uint32_t selector)
{
	if (selector != 0U || (boot_cache_read32(BOOT_CACHE_CONTROL) & BOOT_CACHE_ENABLED) == 0U) {
		return 0U;
	}
	uint32_t start = boot_timer_read_ticks32();

	while ((boot_cache_read32(BOOT_CACHE_PENDING) & BOOT_CACHE_PENDING_MASK) != 0U) {
		if ((uint32_t)(boot_timer_read_ticks32() - start) >= BOOT_CACHE_WAIT_TICKS) {
			break;
		}
	}
	/* The fixed ABI continues after the wait threshold and returns success.
	 * That return alone does not establish that hardware has drained.
	 */
	boot_cache_write32(BOOT_CACHE_INVALIDATE, 1U);
	boot_cache_write32(BOOT_CACHE_CONTROL, 0U);
	return 0U;
}
