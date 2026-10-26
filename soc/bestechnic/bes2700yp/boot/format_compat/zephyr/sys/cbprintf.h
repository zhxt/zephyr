/* Copyright (c) 2020 Nordic Semiconductor ASA */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_FORMAT_CBPRINTF_H_
#define BES_BOOT_FORMAT_CBPRINTF_H_

#include <stdarg.h>
#include <stdint.h>
#include <zephyr/sys/util_macro.h>

/* Declaration-only subset of the pinned public cbprintf.h. No kernel types,
 * packaged arguments, logging backend or application configuration are used.
 */
typedef int (*cbprintf_cb)(int c, void *ctx);
#define Z_CBVPRINTF_PROCESS_FLAG_TAGGED_ARGS BIT(0)

int z_cbvprintf_impl(cbprintf_cb out, void *ctx, const char *format, va_list ap, uint32_t flags);

static inline int cbvprintf(cbprintf_cb out, void *ctx, const char *format, va_list ap)
{
	return z_cbvprintf_impl(out, ctx, format, ap, 0);
}

#endif
