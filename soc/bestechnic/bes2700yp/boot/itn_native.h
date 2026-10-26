/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_ITN_NATIVE_H_
#define BOOT_ITN_NATIVE_H_

#include <stdint.h>

/* Fixed loader ABI: PHY keeps the full page word and requires valid output. */
/* Both cleanup writes are attempted; return the first transfer or cleanup error. */
int pmu_get_efuse_phy(uint32_t page, uint16_t *value);
int pmu_get_efuse(uint32_t page, uint16_t *value);
/* Publish the cache only after every page succeeds; stop on a transfer error. */
void pmu_efuse_init(void);
void pmu_itn_init(void);
void pmu_itn_close_usb_itf(void);
void pmu_itn_open_usb_itf(void);

extern uint8_t boot_itn_efuse_valid;
extern uint16_t boot_itn_efuse_cache[32];

/* Until main PMU replacement, the retained PMU member provides this leaf. */
uint32_t pmu_get_chip_type(void);

#endif /* BOOT_ITN_NATIVE_H_ */
