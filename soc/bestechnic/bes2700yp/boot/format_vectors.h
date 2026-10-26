/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_FORMAT_VECTORS_H_
#define BES_BOOT_FORMAT_VECTORS_H_

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#define BOOT_FORMAT_SELFTEST_CASES 12U
#ifndef BOOT_FORMAT_VECTOR_RESULT
#define BOOT_FORMAT_VECTOR_RESULT(index, passed) ((void)0)
#endif

/* Safe common vectors for both the retained vendor control and new formatter.
 * Unsafe legacy size-zero/read-boundary cases belong to isolated tests only.
 */
static int boot_format_case(size_t capacity, unsigned int wanted, const char *expected,
			    const char *format, ...)
{
	uint8_t buffer[80];
	va_list args;

	if (capacity > 64U) {
		return 0;
	}
	for (unsigned int i = 0U; i < sizeof(buffer); i++) {
		buffer[i] = 0xa5U;
	}
	va_start(args, format);
	int result = vsnprintf((char *)buffer + 8, capacity, format, args);

	va_end(args);
	if (result != (int)wanted) {
		return 0;
	}
	unsigned int written = capacity == 0U ? 0U : (wanted < capacity ? wanted : capacity - 1U);

	for (unsigned int i = 0U; i < sizeof(buffer); i++) {
		uint8_t value = 0xa5U;

		if (i >= 8U && i < 8U + written) {
			value = (uint8_t)expected[i - 8U];
		} else if (capacity != 0U && i == 8U + written) {
			value = 0U;
		}
		if (buffer[i] != value) {
			return 0;
		}
	}
	return 1;
}

static unsigned int boot_format_vectors(void)
{
	unsigned int passed = 0U;
	unsigned int index = 0U;

#define FORMAT_CASE(capacity, expected, ...)                                                       \
	do {                                                                                       \
		int success =                                                                      \
			boot_format_case(capacity, sizeof(expected) - 1U, expected, __VA_ARGS__);  \
		BOOT_FORMAT_VECTOR_RESULT(index, success);                                         \
		index++;                                                                           \
		passed += success;                                                                 \
	} while (0)
	FORMAT_CASE(16U, "boot", "%s", "boot");
	FORMAT_CASE(8U, "%", "%%");
	FORMAT_CASE(16U, "-2147483648", "%d", (-2147483647 - 1));
	FORMAT_CASE(16U, "4294967295", "%u", 4294967295U);
	FORMAT_CASE(32U, "abcdef12/ABCDEF12", "%x/%X", 0xabcdef12U, 0xabcdef12U);
	FORMAT_CASE(32U, "R 00001234: FFFFFFFF", "%c %08X: %08X", 'R', 0x1234U, 0xffffffffU);
	FORMAT_CASE(16U, "       12/", "%9u/", 12U);
	FORMAT_CASE(16U, "boot/-7", "%s/%d", "boot", -7);
	FORMAT_CASE(16U, "    12", "%6u", 12U);
	FORMAT_CASE(16U, "abcdef", "%s", "abcdef");
	FORMAT_CASE(8U, "0/0", "%d/%u", 0, 0U);
	FORMAT_CASE(16U, "-0000123", "%08d", -123);
#undef FORMAT_CASE
	(void)index;
	return passed;
}

#endif
