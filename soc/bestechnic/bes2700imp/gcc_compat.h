/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BES2700IMP_GCC_COMPAT_H
#define BES2700IMP_GCC_COMPAT_H

/* Zephyr's bit helpers use the ACLE intrinsic introduced after GCC 12. */
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ < 14
static inline unsigned int __rbit(unsigned int value)
{
	unsigned int result;

	__asm__("rbit %0, %1" : "=r"(result) : "r"(value));
	return result;
}
#endif

#endif
