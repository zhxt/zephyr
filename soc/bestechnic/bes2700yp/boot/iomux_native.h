/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_IOMUX_NATIVE_H_
#define BES_BOOT_IOMUX_NATIVE_H_

#include <stdint.h>

/* Only the two interfaces retained by the fixed minimal loader are provided. */
void hal_iomux_set_default_config(void);
void hal_iomux_set_uart0(void);
uint32_t hal_cmu_get_aon_revision_id(void);
int hal_memsc_aon_lock(uint32_t selector);
void hal_memsc_aon_unlock(uint32_t selector);

uint32_t boot_iomux_lock(void);
uint32_t boot_iomux_irq_lock(uint32_t caller);
void boot_iomux_irq_unlock(uint32_t key);
uint32_t boot_iomux_read32(uint32_t address);
void boot_iomux_write32(uint32_t address, uint32_t value);
void boot_iomux_sync(void);

extern volatile uint8_t boot_iomux_got_memsc;

#endif /* BES_BOOT_IOMUX_NATIVE_H_ */
