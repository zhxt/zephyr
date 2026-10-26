/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BES2700YP_BOOT_DIAGNOSTICS_H
#define BES2700YP_BOOT_DIAGNOSTICS_H

#include <stdint.h>

#define BOOT_DIAG_BUFFER_BYTES 128U
#define BOOT_DIAG_FORMAT_BYTES 128U
#define BOOT_DIAG_STRING_BYTES 96U
#define BOOT_DIAG_POLL_BUDGET 200000U

/* Compatibility for this locked loader only, not a general vendor trace API. */
int hal_trace_printf(uint32_t attributes, const char *format, ...);
void hal_trace_assert_dump(const char *format, ...) __attribute__((noreturn));
void boot_diag_arm(void);
void boot_diag_disarm(void);
void boot_diag_complete(void);
int boot_diag_selftest(void);

#endif
