/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_CACHE_NATIVE_H_
#define BES_BOOT_CACHE_NATIVE_H_

#include <stdint.h>

/* Fixed selected-build ABI; selectors other than zero have no operations. */
uint8_t hal_cache_enable(uint32_t selector);
uint8_t hal_cache_disable(uint32_t selector);
uint8_t hal_cache_wrap_enable(uint32_t selector);
uint8_t hal_cache_wrap_disable(uint32_t selector);
uint8_t hal_cache_writebuffer_enable(uint32_t selector);
uint8_t hal_cache_writeback_enable(uint32_t selector);

/* Early endpoints execute from Flash before the first boot SRAM copy. */
uint32_t boot_cache_early_read32(uint32_t offset);
void boot_cache_early_write32(uint32_t offset, uint32_t value);
/* Late endpoints and their literals must reside in copied boot SRAM. */
uint32_t boot_cache_read32(uint32_t offset);
void boot_cache_write32(uint32_t offset, uint32_t value);
uint32_t boot_timer_read_ticks32(void);

#define BOOT_CACHE_CONTROL 0U
#define BOOT_CACHE_PREPARE 4U
#define BOOT_CACHE_PENDING 0x30U
#define BOOT_CACHE_INVALIDATE 0x38U
#define BOOT_CACHE_ENABLED 1U
#define BOOT_CACHE_WRAP 2U
#define BOOT_CACHE_PENDING_MASK 0x3ffU
#define BOOT_CACHE_WAIT_TICKS 32U

#endif /* BES_BOOT_CACHE_NATIVE_H_ */
