/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_WRAPPERS_ABI_H_
#define BES_BOOT_WRAPPERS_ABI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct boot_uart_config {
	uint8_t parity;
	uint8_t stop;
	uint8_t data;
	uint8_t flow;
	uint8_t rx_level;
	uint8_t tx_level;
	uint32_t baud;
	bool dma_rx : 1;
	bool dma_tx : 1;
	bool dma_rx_stop_on_err : 1;
};

_Static_assert(sizeof(struct boot_uart_config) == 16,
	       "locked UART config size");
_Static_assert(_Alignof(struct boot_uart_config) == 4,
	       "locked UART config alignment");
_Static_assert(offsetof(struct boot_uart_config, parity) == 0,
	       "UART parity offset");
_Static_assert(offsetof(struct boot_uart_config, stop) == 1,
	       "UART stop offset");
_Static_assert(offsetof(struct boot_uart_config, data) == 2,
	       "UART data offset");
_Static_assert(offsetof(struct boot_uart_config, flow) == 3,
	       "UART flow offset");
_Static_assert(offsetof(struct boot_uart_config, rx_level) == 4,
	       "UART RX offset");
_Static_assert(offsetof(struct boot_uart_config, tx_level) == 5,
	       "UART TX offset");
_Static_assert(offsetof(struct boot_uart_config, baud) == 8,
	       "UART baud offset");

/* Fixed AAPCS bridge declarations for this release. */
void hal_iomux_set_uart0(void);
int hal_uart_open(uint32_t id, const struct boot_uart_config *config);
int pmu_open(void);
void pmu_wdt_stop(void);
int hal_wdt_stop(uint32_t id);
int hal_cmu_sys_set_freq(uint32_t frequency);
uint32_t hal_cmu_sys_get_freq(void);
void hal_sys_timer_open(void);
int hal_cache_disable(uint32_t id);

#endif /* BES_BOOT_WRAPPERS_ABI_H_ */
