/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BES2700YP_BOOT_PLATFORM_H
#define BES2700YP_BOOT_PLATFORM_H

#include <stdint.h>

/* BTH bootstrap only. Calls occur after the selected early startup has prepared
 * bootstrap data/BSS. The vendor experiment retains that startup dependency.
 * Zephyr takes over UART and the core after handoff; no runtime services exist.
 */
int32_t bes_boot_uart_open(void);
int32_t bes_boot_pmu_open(void);
void bes_boot_watchdogs_stop(void);
int32_t bes_boot_clock_24m(void);
uint32_t bes_boot_crystal_hz(void);
uint32_t bes_boot_fast_timer_hz(void);
void bes_boot_timer_open(void);
void bes_boot_dcache_disable(void);
void bes_boot_icache_disable(void);

#endif
