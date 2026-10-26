/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include <limits.h>
#include <stdbool.h>
#include <stdio.h>

int zephyr_boot_vsnprintf(char *str, size_t size, const char *format, va_list args);

/* The loader exposes an explicitly bounded subset, not every C printf form.
 * Validate the complete format before consuming arguments. In particular nano
 * must never continue past an unsupported conversion with a shifted va_list.
 * Actual conversion and bounded emission remain in the public Zephyr code.
 */
static bool decimal(const char **cursor)
{
	unsigned int value = 0U;

	while (**cursor >= '0' && **cursor <= '9') {
		value = value * 10U + (unsigned int)(*(*cursor)++ - '0');
		if (value > 4096U) {
			return false;
		}
	}
	return true;
}

static bool supported(const char *format)
{
	while (*format != '\0') {
		bool zero = false;
		bool precision = false;
		unsigned int length = 0U;

		if (*format++ != '%') {
			continue;
		}
		if (*format == '%') {
			format++;
			continue;
		}
		while (*format == '0' || *format == '-') {
			zero |= *format++ == '0';
		}
		if (!decimal(&format)) {
			return false;
		}
		if (*format == '.') {
			precision = true;
			format++;
			if (*format == '*') {
				format++;
			} else if (!decimal(&format)) {
				return false;
			}
		}
		while (*format == 'l' && length < 2U) {
			length++;
			format++;
		}
		switch (*format++) {
		case 'd':
		case 'i':
		case 'u':
		case 'x':
		case 'X':
			if (precision) {
				return false;
			}
			break;
		case 's':
			if (length != 0U || zero) {
				return false;
			}
			break;
		case 'c':
		case 'p':
			if (precision || length != 0U || zero) {
				return false;
			}
			break;
		default:
			return false;
		}
	}
	return true;
}

int vsnprintf(char *str, size_t size, const char *format, va_list args)
{
	if (size > INT_MAX || format == NULL || (size != 0U && str == NULL) || !supported(format)) {
		if (str != NULL && size != 0U) {
			str[0] = '\0';
		}
		return -1;
	}
	return zephyr_boot_vsnprintf(str, size, format, args);
}
