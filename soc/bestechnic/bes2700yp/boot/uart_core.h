/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_UART_CORE_H_
#define BES_BOOT_UART_CORE_H_

#include <stdint.h>

int boot_uart_core_quiesce(void);
int boot_uart_core_configure(void);
int boot_uart_poll_open(void);
void boot_uart_unexpected_irq(void);

#ifdef BES_BOOT_UART_TAKEOVER
int boot_uart_core_enabled(void);
int boot_uart_core_disable(void);
int hal_cmu_reset_set(uint32_t module);
int hal_cmu_clock_disable(uint32_t module);
#endif

/* Fixed release ABI; hardware operations remain delegated to HAL helpers. */
uint32_t hal_cmu_clock_get_status(uint32_t module);
int hal_cmu_clock_enable(uint32_t module);
int hal_cmu_reset_clear(uint32_t module);
int hal_cmu_uart0_set_freq(uint32_t frequency);
int hal_uart_opened(uint32_t id);
int hal_uart_close(uint32_t id);
void hal_iomux_set_uart0(void);

#ifdef BOOT_UART_CORE_HOST
uint32_t boot_uart_host_read(uint32_t offset);
void boot_uart_host_write(uint32_t offset, uint32_t value);
#endif

#endif /* BES_BOOT_UART_CORE_H_ */
