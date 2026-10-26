/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "arch.h"
#include "boot_vectors.h"

extern const uint8_t __flashx_start_addr__[];
extern const uint8_t __flashx_text_end__[];

static void mismatch(struct boot_vectors_report *report, unsigned int bit, uint32_t slot,
		     uintptr_t expected, uintptr_t actual)
{
	if (report->fail_mask == 0U) {
		report->first_slot = slot;
		report->expected = expected;
		report->actual = actual;
	}
	report->fail_mask |= 1U << bit;
}

void boot_vectors_capture(struct boot_vectors_report *report, uintptr_t uart_handler)
{
	const uintptr_t core[BOOT_CORE_VECTORS] = {
		(uintptr_t)&__StackTop,        (uintptr_t)Reset_Handler,
		(uintptr_t)NMI_Handler,        (uintptr_t)HardFault_Handler,
		(uintptr_t)MemManage_Handler,  (uintptr_t)BusFault_Handler,
		(uintptr_t)UsageFault_Handler, (uintptr_t)SecureFault_Handler,
		(uintptr_t)NMI_Handler,        (uintptr_t)NMI_Handler,
		(uintptr_t)NMI_Handler,        (uintptr_t)SVC_Handler,
		(uintptr_t)DebugMon_Handler,   (uintptr_t)NMI_Handler,
		(uintptr_t)PendSV_Handler,     (uintptr_t)SysTick_Handler,
	};
	uintptr_t table = (uintptr_t)__vector_table_start;
	uintptr_t fallback = (uintptr_t)NMI_Handler;

	report->fail_mask = 0U;
	report->vtor = SCB->VTOR;
	report->first_slot = UINT32_MAX;
	report->expected = 0U;
	report->actual = 0U;
	if (report->vtor != table) {
		mismatch(report, 0U, UINT32_MAX, table, report->vtor);
	}
	if ((report->vtor & (BOOT_VECTOR_ALIGNMENT - 1U)) != 0U) {
		mismatch(report, 1U, UINT32_MAX, table, report->vtor);
	}
	if (__vector_table_start[0] != core[0]) {
		mismatch(report, 2U, 0U, core[0], __vector_table_start[0]);
	}
	for (unsigned int i = 1U; i < BOOT_CORE_VECTORS; i++) {
		if (__vector_table_start[i] != core[i]) {
			mismatch(report, 3U, i, core[i], __vector_table_start[i]);
		}
	}
	/* Zero selects the initial all-default table. After UART setup, only its
	 * registered entry may differ; require a nondefault Thumb entry within
	 * linked loader Flash code. Do not call or dereference the handler.
	 */
	if (uart_handler != 0U &&
	    ((uart_handler & 1U) == 0U || uart_handler == fallback ||
	     (uart_handler & ~(uintptr_t)1U) < (uintptr_t)__flashx_start_addr__ ||
	     (uart_handler & ~(uintptr_t)1U) >= (uintptr_t)__flashx_text_end__)) {
		/* expected=0 denotes a handler-validity failure, not a NULL vector. */
		mismatch(report, 4U, BOOT_VECTOR_UART_SLOT, 0U, uart_handler);
	}
	for (unsigned int i = BOOT_CORE_VECTORS; i < BOOT_VECTOR_COUNT; i++) {
		uintptr_t expected =
			uart_handler != 0U && i == BOOT_VECTOR_UART_SLOT ? uart_handler : fallback;

		if (__vector_table_start[i] != expected) {
			mismatch(report, 4U, i, expected, __vector_table_start[i]);
		}
	}
	if (__get_IPSR() != 0U || NVIC_GetCurrentActiveIRQ() != -16) {
		mismatch(report, 5U, UINT32_MAX, 0U, __get_IPSR());
	}
}

unsigned int boot_vectors_check(void)
{
	struct boot_vectors_report report;
	unsigned int passed = BOOT_VECTOR_SELFTEST_CHECKS;

	boot_vectors_capture(&report, 0U);
	for (unsigned int i = 0U; i < BOOT_VECTOR_SELFTEST_CHECKS; i++) {
		passed -= (report.fail_mask >> i) & 1U;
	}
	return passed;
}

void boot_vectors_before_uart(struct boot_vectors_state *state)
{
	state->uart_handler = 0U;
	boot_vectors_capture(&state->points[0], 0U);
}

void boot_vectors_after_uart(struct boot_vectors_state *state)
{
	state->uart_handler = __vector_table_start[BOOT_VECTOR_UART_SLOT];
	boot_vectors_capture(&state->points[1], state->uart_handler);
}
