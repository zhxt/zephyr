/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BES2700YP_BOOT_DIAGNOSTICS_PORT_H
#define BES2700YP_BOOT_DIAGNOSTICS_PORT_H

#include <stdint.h>

/* All production primitives execute from the first copied boot SRAM region. */
uint32_t diag_irq_lock(void);
void diag_irq_unlock(uint32_t key);
int diag_tx_full(void);
void diag_tx_byte(char byte);
__attribute__((noreturn)) void diag_halt(void);

#endif
