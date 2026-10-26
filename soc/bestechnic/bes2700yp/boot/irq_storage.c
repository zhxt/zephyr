/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdint.h>

/* Preserve the storage ABI used by the existing hardware backend. */
uint32_t irq_masked_addr[2]
	__attribute__((section(".boot_bss.irq_storage")));

_Static_assert(sizeof(irq_masked_addr) == 8, "IRQ storage ABI size");

_Static_assert(__alignof__(irq_masked_addr) >= 4, "IRQ storage alignment");
