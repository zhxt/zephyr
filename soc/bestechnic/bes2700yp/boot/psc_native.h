/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_PSC_NATIVE_H_
#define BES_BOOT_PSC_NATIVE_H_

#include <stdint.h>

/* Fixed selected-build interfaces; numeric fields have no public BES meaning. */
void hal_cmu_sys_ram_active_cfg(uint32_t map, uint32_t mode);
void hal_cmu_sys_ram_sleep_cfg(uint32_t map, uint32_t mode);
void hal_psc_sys_disable(uint32_t user);
void hal_psc_sys_m55_disable(void);
void hal_psc_sys_dsp_disable(void);
void hal_psc_sens_disable(void);
void hal_psc_display_disable(uint32_t user);
void hal_psc_storage_disable(uint32_t user);
void hal_psc_init(void);
void hal_psc_core_init_ram_ret_mask(void);
void hal_psc_ram_ret_during_reset(void);

uint32_t boot_psc_read32(uint32_t address);
void boot_psc_write32(uint32_t address, uint32_t value);
uint32_t boot_psc_irq_lock(uint32_t caller);
void boot_psc_irq_unlock(uint32_t key);
void boot_psc_sync(void);

extern volatile uint16_t boot_psc_bth_shr_pu, boot_psc_bth_shr_pd;
extern volatile uint16_t boot_psc_m55_tcm, boot_psc_m55_shared;
extern volatile uint16_t boot_psc_dsp_tcm, boot_psc_dsp_shared;
extern volatile uint8_t boot_psc_sys_map;
extern volatile uint8_t boot_psc_display_map, boot_psc_storage_map;

#endif /* BES_BOOT_PSC_NATIVE_H_ */
