/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BES2700YP_BOOT_RUNTIME_H
#define BES2700YP_BOOT_RUNTIME_H

/* All functions execute from Flash before normal data/BSS initialization. */
void boot_cpu_init(void);
void boot_crt_copy_boot_sram(void);
void boot_crt_copy_other_sram(void);
void boot_crt_init(void);

/* Explicit retained hardware boundary; these are not open hardware drivers. */
void bes_boot_early_cache_prepare(void);
void bes_boot_early_clock_prepare(void);

#endif
