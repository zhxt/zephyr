/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */

#ifndef BES_BOOT_TIMER_WAIT_H_
#define BES_BOOT_TIMER_WAIT_H_

#include <stdint.h>

/* The backend must provide an advancing 32-bit up-counter. It and its data
 * must be available wherever the early SRAM caller can execute.
 */
uint32_t boot_timer_read_ticks32(void);

/* Wait in raw counter ticks without changing clocks or interrupt state.
 * The backend must remain in the same counter domain for the whole wait.
 * Polling must observe the threshold before a full counter period passes;
 * unsigned subtraction alone cannot detect elapsed multiple wraps.
 */
void boot_timer_wait_up32(uint32_t ticks);

#endif /* BES_BOOT_TIMER_WAIT_H_ */
