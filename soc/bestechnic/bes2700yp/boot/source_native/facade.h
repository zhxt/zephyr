/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BES_BOOT_SOURCE_FACADE_H_
#define BES_BOOT_SOURCE_FACADE_H_

#include <stdint.h>

/* Local source facade; no SDK header or external binary ABI is imported. */
int bes2700yp_uart_open(void);
int bes2700yp_pmu_open(void);
void bes2700yp_watchdogs_stop(void);
int bes2700yp_bth_clock_24m(void);
uint32_t bes2700yp_crystal_hz(void);
uint32_t bes2700yp_fast_timer_hz(void);
void bes2700yp_timer_open(void);
void bes2700yp_dcache_disable(void);
void bes2700yp_icache_disable(void);

#endif /* BES_BOOT_SOURCE_FACADE_H_ */
