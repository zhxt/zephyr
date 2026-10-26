/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "nor_native.h"

#define EARLY(name) __attribute__((section(".boot_bss." #name)))

EARLY(nor_div_read) uint8_t boot_nor_div_read[2];
EARLY(nor_div_others) uint8_t boot_nor_div_others[2];
EARLY(nor_preferred_idx_adj) uint8_t boot_nor_preferred_idx_adj[2];
EARLY(nor_mrb_status) uint32_t boot_nor_mrb_status[2];
