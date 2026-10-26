/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

/* Nine wrappers adapted from the locked, local Apache port/bes2700yp/hw.c.
 * This is a bridge to the fixed HAL binary ABI, not a hardware implementation.
 */
#include "wrappers_abi.h"

int bes2700yp_uart_open(void)
{
	const struct boot_uart_config uart = {
		.parity = 0, .stop = 0, .data = 3, .flow = 0,
		.rx_level = 2, .tx_level = 2,
		.baud = 1152000, .dma_rx = false, .dma_tx = false,
	};

	hal_iomux_set_uart0();
	return hal_uart_open(0, &uart);
}

int bes2700yp_pmu_open(void)
{
	return pmu_open();
}

void bes2700yp_watchdogs_stop(void)
{
	pmu_wdt_stop();
	hal_wdt_stop(0);
}

int bes2700yp_bth_clock_24m(void)
{
	return hal_cmu_sys_set_freq(6) || hal_cmu_sys_get_freq() != 6;
}

uint32_t bes2700yp_crystal_hz(void)
{
	/* The original wrapper inlines this constant for dual_v1_24m_t2. */
	return 24000000;
}

uint32_t bes2700yp_fast_timer_hz(void)
{
	return 6000000;
}

void bes2700yp_timer_open(void)
{
	hal_sys_timer_open();
}

void bes2700yp_dcache_disable(void)
{
	hal_cache_disable(1);
}

void bes2700yp_icache_disable(void)
{
	hal_cache_disable(0);
}
