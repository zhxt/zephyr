/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_PMU_BOOT_NATIVE_H_
#define BOOT_PMU_BOOT_NATIVE_H_

#include <stdint.h>

extern uint16_t boot_pmu_boot_reason;
extern uint8_t boot_pmu_boot_first_pwr_up;
extern uint16_t boot_pmu_chg_ctx;

void pmu_boot_init(void);

#endif /* BOOT_PMU_BOOT_NATIVE_H_ */
