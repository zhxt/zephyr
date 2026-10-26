/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

/* Register configuration adapted from the pinned Apache uart_bes2700.c.
 * Clock/reset/pin ownership and legacy cleanup are outside this module.
 */
#include "uart_core.h"
#ifndef BOOT_UART_CORE_HOST
#include "arch.h"
#endif

#define UART_BASE 0x4000b000U
#define UART_ECR 0x04U
#define UART_FR 0x18U
#define UART_IBRD 0x24U
#define UART_FBRD 0x28U
#define UART_LCR 0x2cU
#define UART_CR 0x30U
#define UART_IFLS 0x34U
#define UART_IMSC 0x38U
#define UART_ICR 0x44U
#define UART_DMACR 0x48U
#define UART_OVSAMP 0x4cU
#define UART_OVSAMPST 0x50U
#define UART_RXEXT 0x54U
#define UART_BUSY (1U << 3)
#define UART_POLL_BUDGET 1000000U

static uint32_t rd(uint32_t offset)
{
#ifdef BOOT_UART_CORE_HOST
	return boot_uart_host_read(offset);
#else
	return *(__IOM uint32_t *)(UART_BASE + offset);
#endif
}

static void wr(uint32_t offset, uint32_t value)
{
#ifdef BOOT_UART_CORE_HOST
	boot_uart_host_write(offset, value);
#else
	*(__IOM uint32_t *)(UART_BASE + offset) = value;
#endif
}

int boot_uart_core_quiesce(void)
{
	/* Reject DMA requests before UART writes. DMA channel ownership and
	 * cleanup remain outside this register configuration module.
	 */
	if ((rd(UART_DMACR) & 7U) != 0U) {
		return -1;
	}
	for (uint32_t budget = UART_POLL_BUDGET; budget > 0U; --budget) {
		if ((rd(UART_FR) & UART_BUSY) == 0U) {
			return 0;
		}
	}
	return -2;
}

int boot_uart_core_configure(void)
{
	int rc = boot_uart_core_quiesce();

	if (rc != 0) {
		return rc;
	}
	wr(UART_IMSC, 0U);
	wr(UART_CR, 0U);
	wr(UART_LCR, 0U);
	wr(UART_DMACR, 0U);
	wr(UART_ECR, 0U);
	wr(UART_ICR, 0x7ffU);
	wr(UART_OVSAMP, 15U);
	wr(UART_OVSAMPST, 8U);
	/* round(24 MHz * 4 / 1152000) = 83: 16x oversampling, 1 + 19/64. */
	wr(UART_IBRD, 1U);
	wr(UART_FBRD, 19U);
	wr(UART_RXEXT, 3U);
	wr(UART_IFLS, 0U);
	wr(UART_LCR, 0x70U);
	wr(UART_CR, 0x301U);
	if (rd(UART_IMSC) != 0U || rd(UART_DMACR) != 0U ||
	    rd(UART_IBRD) != 1U || rd(UART_FBRD) != 19U ||
	    rd(UART_OVSAMP) != 15U || rd(UART_OVSAMPST) != 8U ||
	    rd(UART_RXEXT) != 3U || rd(UART_IFLS) != 0U ||
	    rd(UART_LCR) != 0x70U || rd(UART_CR) != 0x301U) {
		wr(UART_IMSC, 0U);
		wr(UART_CR, 0U);
		return -3;
	}
	return 0;
}

#ifdef BES_BOOT_UART_TAKEOVER
/* UARTEN and FEN semantics follow the Apache Zephyr PL011 helpers. */
int boot_uart_core_enabled(void)
{
	return (rd(UART_CR) & 1U) != 0U;
}

int boot_uart_core_disable(void)
{
	int rc = boot_uart_core_quiesce();

	if (rc != 0) {
		return rc;
	}
	wr(UART_CR, rd(UART_CR) & ~1U);
	wr(UART_LCR, rd(UART_LCR) & ~(1U << 4));
	if ((rd(UART_CR) & 1U) != 0U || (rd(UART_LCR) & (1U << 4)) != 0U) {
		return -3;
	}
	return 0;
}
#endif
