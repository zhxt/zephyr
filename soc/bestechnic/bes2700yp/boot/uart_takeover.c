/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "uart_core.h"
#ifndef BOOT_UART_CORE_HOST
#include "arch.h"
#else
#include "tests/uart_takeover_host.h"
#endif

#define UART0_IRQ ((IRQn_Type)17)
#define UART0_MODULE 36U
#define UART0_APB 25U
#define UART0_XTAL_SOURCE 0U

void boot_uart_unexpected_irq(void)
{
	__disable_irq();
	__DSB();
	for (;;) {
		__WFI();
	}
}

int boot_uart_poll_open(void)
{
	uint32_t clock;
	int rc;

	NVIC_DisableIRQ(UART0_IRQ);
	__DSB();
	__ISB();
	clock = hal_cmu_clock_get_status(UART0_APB);
	if (clock > 1U) {
		return -5;
	}
	if (clock == 1U) {
		rc = boot_uart_core_quiesce();
		if (rc != 0) {
			return rc;
		}
	}
	hal_iomux_set_uart0();
	clock = hal_cmu_clock_get_status(UART0_APB);
	if (clock > 1U) {
		return -5;
	}
	if (clock == 1U && boot_uart_core_enabled() != 0) {
		rc = boot_uart_core_disable();
		if (rc != 0) {
			return rc;
		}
		__DSB();
		if (hal_cmu_reset_set(UART0_APB) != 0 ||
		    hal_cmu_reset_set(UART0_MODULE) != 0 ||
		    hal_cmu_clock_disable(UART0_APB) != 0 ||
		    hal_cmu_clock_disable(UART0_MODULE) != 0) {
			return -4;
		}
	}
	if (hal_cmu_clock_enable(UART0_MODULE) != 0 ||
	    hal_cmu_clock_enable(UART0_APB) != 0 ||
	    hal_cmu_reset_clear(UART0_MODULE) != 0 ||
	    hal_cmu_reset_clear(UART0_APB) != 0 ||
	    hal_cmu_uart0_set_freq(UART0_XTAL_SOURCE) != 0) {
		return -5;
	}
	rc = boot_uart_core_configure();
	if (rc != 0) {
		return rc;
	}
	NVIC_SetVector(UART0_IRQ,
		       (uint32_t)(uintptr_t)boot_uart_unexpected_irq);
	NVIC_ClearPendingIRQ(UART0_IRQ);
	__DSB();
	__ISB();
	return 0;
}
