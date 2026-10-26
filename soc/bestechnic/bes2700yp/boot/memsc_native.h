/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_MEMSC_NATIVE_H_
#define BES_BOOT_MEMSC_NATIVE_H_

#include <stdint.h>

/* Preserve the selected 32-bit argument/return ABI, including raw high bits. */
int hal_memsc_aon_lock(uint32_t selector);
void hal_memsc_aon_unlock(uint32_t selector);

/* Both endpoints and their literals execute from copied boot SRAM. */
uint32_t boot_memsc_read32(uint32_t offset);
void boot_memsc_write32(uint32_t offset, uint32_t value);

#define BOOT_MEMSC_SLOT_COUNT 16U
#define BOOT_MEMSC_SLOT_STRIDE 4U

#endif /* BES_BOOT_MEMSC_NATIVE_H_ */
