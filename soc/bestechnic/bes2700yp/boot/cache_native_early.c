/* SPDX-License-Identifier: Apache-2.0 */

#include "cache_native.h"

__attribute__((section(".boot_text_flash.hal_cache_enable")))
uint8_t hal_cache_enable(uint32_t selector)
{
	if (selector == 0U) {
		if ((boot_cache_early_read32(BOOT_CACHE_CONTROL) & BOOT_CACHE_ENABLED) != 0U) {
			boot_cache_early_write32(BOOT_CACHE_INVALIDATE, 1U);
		} else {
			boot_cache_early_write32(BOOT_CACHE_PREPARE, 1U);
			boot_cache_early_write32(BOOT_CACHE_CONTROL, BOOT_CACHE_ENABLED);
		}
	}
	return 0U;
}

/* These two selected binary interfaces do not operate hardware. This does
 * not define write-buffer or write-back support for other BES builds.
 */
__attribute__((section(".boot_text_flash.hal_cache_writebuffer_enable")))
uint8_t hal_cache_writebuffer_enable(uint32_t selector)
{
	(void)selector;
	return 0U;
}

__attribute__((section(".boot_text_flash.hal_cache_writeback_enable")))
uint8_t hal_cache_writeback_enable(uint32_t selector)
{
	(void)selector;
	return 0U;
}
