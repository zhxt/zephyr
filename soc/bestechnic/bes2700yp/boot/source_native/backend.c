/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "facade.h"
#include "platform.h"
#include "uart_core.h"

int32_t bes_boot_uart_open(void)
{
	return boot_uart_poll_open();
}

int32_t bes_boot_pmu_open(void)
{
	return bes2700yp_pmu_open();
}

void bes_boot_watchdogs_stop(void)
{
	bes2700yp_watchdogs_stop();
}

int32_t bes_boot_clock_24m(void)
{
	return bes2700yp_bth_clock_24m();
}

uint32_t bes_boot_crystal_hz(void)
{
	return bes2700yp_crystal_hz();
}

uint32_t bes_boot_fast_timer_hz(void)
{
	return bes2700yp_fast_timer_hz();
}

void bes_boot_timer_open(void)
{
	bes2700yp_timer_open();
}

void bes_boot_dcache_disable(void)
{
	bes2700yp_dcache_disable();
}

void bes_boot_icache_disable(void)
{
	bes2700yp_icache_disable();
}
