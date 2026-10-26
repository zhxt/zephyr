/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "arch.h"
#include "boot_contract.h"
#include "diagnostics_port.h"

#define SRAM __attribute__((section(".boot_text_sram.diag_port"), noinline))

SRAM uint32_t diag_irq_lock(void)
{
	uint32_t key = __get_PRIMASK();

	__disable_irq();
	return key;
}

SRAM void diag_irq_unlock(uint32_t key)
{
	__set_PRIMASK(key);
}

SRAM int diag_tx_full(void)
{
	return (BTH_REG(BTH_UART_BASE + 0x18U) & (1U << 5)) != 0U;
}

SRAM void diag_tx_byte(char byte)
{
	BTH_REG(BTH_UART_BASE) = (uint8_t)byte;
}

SRAM void diag_halt(void)
{
	__DSB();
	for (;;) {
		__WFI();
	}
}
