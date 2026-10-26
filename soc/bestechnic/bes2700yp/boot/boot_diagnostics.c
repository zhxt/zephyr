/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

#include "boot_diagnostics.h"
#include "diagnostics_port.h"

#define SRAM __attribute__((section(".boot_text_sram.diagnostics"), noinline))
#define FLASH __attribute__((section(".text.boot_diagnostics"), noinline))
#define NO_NEWLINE (1U << 15)

/* Initialized by the first SRAM copy/zero, before any reachable diagnostic
 * caller. M55 release and early cache roots have no trace/assert references;
 * the build audit checks that boundary against the actual linked image.
 */
static struct {
	uint32_t ready;
	uint32_t busy;
	uint32_t fatal;
} boot_diag_state __attribute__((section(".boot_bss.diagnostics")));

static const char early_fatal[] __attribute__((section(".boot_data.diagnostics"))) =
	"zephyr_bth diag_fatal early\r\n";

SRAM void boot_diag_arm(void)
{
	uint32_t key = diag_irq_lock();

	boot_diag_state.ready = 1U;
	diag_irq_unlock(key);
}

SRAM void boot_diag_disarm(void)
{
	uint32_t key = diag_irq_lock();

	boot_diag_state.ready = 0U;
	diag_irq_unlock(key);
}

/* Full formatting is safe only after the hardware/Flash preparation window
 * and IRQ teardown. Until then a fatal path emits an SRAM-resident marker.
 */
SRAM void boot_diag_complete(void)
{
	uint32_t key = diag_irq_lock();

	if (boot_diag_state.ready == 1U) {
		boot_diag_state.ready = 2U;
	}
	diag_irq_unlock(key);
}

/* Clamp every string scan without changing argument order. This fixed grammar
 * covers all retained HAL formats. No '*' or floating-point arguments, %n,
 * integer precision, arbitrary widths or unbounded format scans are accepted.
 */
FLASH static int boot_diag_format(char *out, const char *format)
{
	unsigned int n = 0U;
	unsigned int used = 0U;
	unsigned int fields = 0U;

	if (format == NULL) {
		return -1;
	}
	while (n < BOOT_DIAG_FORMAT_BYTES && format[n] != '\0') {
		if (used + 16U >= 2U * BOOT_DIAG_FORMAT_BYTES) {
			return -1;
		}
		out[used++] = format[n++];
		if (out[used - 1U] != '%') {
			continue;
		}
		if (n == BOOT_DIAG_FORMAT_BYTES) {
			return -1;
		}
		if (format[n] == '%') {
			out[used++] = format[n++];
			continue;
		}
		if (++fields > 16U) {
			return -1;
		}
		unsigned int width = 0U;
		unsigned int zero = format[n] == '0';
		unsigned int precision = BOOT_DIAG_STRING_BYTES;
		unsigned int has_precision = 0U;
		unsigned int length = 0U;

		while (n < BOOT_DIAG_FORMAT_BYTES && format[n] >= '0' && format[n] <= '9') {
			width = width * 10U + (unsigned int)(format[n] - '0');
			out[used++] = format[n++];
			if (width > 32U || used + 16U >= 2U * BOOT_DIAG_FORMAT_BYTES) {
				return -1;
			}
		}
		if (n < BOOT_DIAG_FORMAT_BYTES && format[n] == '.') {
			has_precision = 1U;
			precision = 0U;
			n++;
			while (n < BOOT_DIAG_FORMAT_BYTES && format[n] >= '0' && format[n] <= '9') {
				/* Saturation also bounds long decimal precision spellings. */
				if (precision < BOOT_DIAG_STRING_BYTES) {
					precision = precision * 10U +
						    (unsigned int)(format[n] - '0');
				}
				n++;
			}
			if (precision > BOOT_DIAG_STRING_BYTES) {
				precision = BOOT_DIAG_STRING_BYTES;
			}
		}
		while (n < BOOT_DIAG_FORMAT_BYTES && format[n] == 'l' && length < 2U) {
			out[used++] = format[n++];
			length++;
		}
		if (n == BOOT_DIAG_FORMAT_BYTES) {
			return -1;
		}
		char conversion = format[n++];

		if (conversion == 's') {
			if (width != 0U || zero != 0U || length != 0U) {
				return -1;
			}
			out[used++] = '.';
			out[used++] = (char)('0' + precision / 10U);
			out[used++] = (char)('0' + precision % 10U);
		} else if (has_precision != 0U ||
			   !((conversion == 'd' || conversion == 'i' || conversion == 'u' ||
			      conversion == 'x' || conversion == 'X') ||
			     ((conversion == 'c' || conversion == 'p') &&
			      length == 0U && zero == 0U))) {
			return -1;
		}
		out[used++] = conversion;
	}
	if (n == BOOT_DIAG_FORMAT_BYTES) {
		return -1;
	}
	out[used] = '\0';
	return 0;
}

SRAM static int boot_diag_emit(const char *bytes, unsigned int count, unsigned int *budget)
{
	for (unsigned int i = 0U; i < count; i++) {
		for (;;) {
			if (*budget == 0U) {
				return -1;
			}
			(*budget)--;
			if (!diag_tx_full()) {
				break;
			}
		}
		diag_tx_byte(bytes[i]);
	}
	return (int)count;
}

FLASH static int boot_diag_print(uint32_t attributes, const char *format, va_list args, int fatal)
{
	char selected[2U * BOOT_DIAG_FORMAT_BYTES];
	char buffer[BOOT_DIAG_BUFFER_BYTES];
	unsigned int budget = BOOT_DIAG_POLL_BUDGET;

	if (fatal != 0 && boot_diag_emit("zephyr_bth diag_fatal ",
				  sizeof("zephyr_bth diag_fatal ") - 1U,
			       &budget) < 0) {
		return -1;
	}
	if (boot_diag_format(selected, format) != 0) {
		if (fatal != 0) {
			(void)boot_diag_emit("invalid-format\r\n",
					     sizeof("invalid-format\r\n") - 1U,
				   &budget);
		}
		return -1;
	}
	int count = vsnprintf(buffer, sizeof(buffer), selected, args);

	if (count < 0) {
		return -1;
	}
	if ((unsigned int)count >= sizeof(buffer)) {
		count = (int)sizeof(buffer) - 1;
	}
	if (boot_diag_emit(buffer, (unsigned int)count, &budget) < 0) {
		return -1;
	}
	if ((attributes & NO_NEWLINE) == 0U) {
		if (boot_diag_emit("\r\n", 2U, &budget) < 0) {
			return -1;
		}
		count += 2;
	}
	return count;
}

SRAM int hal_trace_printf(uint32_t attributes, const char *format, ...)
{
	uint32_t key = diag_irq_lock();
	int result = 0;

	if (boot_diag_state.ready != 0U && boot_diag_state.busy == 0U &&
	    boot_diag_state.fatal == 0U) {
		va_list args;

		boot_diag_state.busy = 1U;
		va_start(args, format);
		result = boot_diag_print(attributes, format, args, 0);
		va_end(args);
		boot_diag_state.busy = 0U;
	}
	diag_irq_unlock(key);
	return result;
}

SRAM void hal_trace_assert_dump(const char *format, ...)
{
	(void)diag_irq_lock();
	boot_diag_state.fatal = 1U;
	if (boot_diag_state.ready != 0U && boot_diag_state.busy == 0U) {
		boot_diag_state.busy = 1U;
		if (boot_diag_state.ready == 2U) {
			va_list args;

			va_start(args, format);
			(void)boot_diag_print(0U, format, args, 1);
			va_end(args);
		} else {
			unsigned int budget = BOOT_DIAG_POLL_BUDGET;

			(void)boot_diag_emit(early_fatal, sizeof(early_fatal) - 1U, &budget);
		}
	}
	/* The pre-UART path has no Flash helper, string, peripheral or RTOS call. */
	diag_halt();
}

FLASH int boot_diag_selftest(void)
{
	/* Deferred until after the timer capture window. Fatal-path injection is
	 * executed separately by the host and Cortex-M33 object tests.
	 */
	int count = hal_trace_printf(0U, "zephyr_bth diagnostics_probe value=%u", 4660U);

	return count == (int)sizeof("zephyr_bth diagnostics_probe value=4660\r\n") - 1 ? 0 : -1;
}
