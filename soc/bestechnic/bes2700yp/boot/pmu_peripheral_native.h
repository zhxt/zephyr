/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_PMU_PERIPHERAL_NATIVE_H_
#define BOOT_PMU_PERIPHERAL_NATIVE_H_

#include <stdint.h>

void pmu_wdt_stop(void);
void pmu_rf_ana_init(void);
void pmu_usb_config(uint32_t enabled);

#endif /* BOOT_PMU_PERIPHERAL_NATIVE_H_ */
