/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "boot_contract.h"
#include "boot_vectors.h"

int boot_vectors_selftest(struct boot_vectors_state *state)
{
	static const char *const phases[] = {"before_uart", "after_uart", "before_handoff"};
	unsigned int passed = 0U;
	unsigned int checks = BOOT_VECTOR_PHASE_COUNT * BOOT_VECTOR_SELFTEST_CHECKS;

	boot_vectors_capture(&state->points[2], state->uart_handler);
	for (unsigned int i = 0U; i < BOOT_VECTOR_PHASE_COUNT; i++) {
		const struct boot_vectors_report *point = &state->points[i];
		unsigned int failed = 0U;

		for (unsigned int bit = 0U; bit < BOOT_VECTOR_SELFTEST_CHECKS; bit++) {
			failed += (point->fail_mask >> bit) & 1U;
		}
		passed += BOOT_VECTOR_SELFTEST_CHECKS - failed;
		bth_puts("zephyr_bth vectors_check version=2 phase=");
		bth_puts(phases[i]);
		bth_puts(" checks=6 pass=");
		bth_dec(BOOT_VECTOR_SELFTEST_CHECKS - failed);
		bth_puts(" fail=");
		bth_dec(failed);
		bth_field(" fail_mask=", point->fail_mask);
		bth_field(" vtor=", point->vtor);
		bth_field(" first_slot=", point->first_slot);
		bth_field(" expected=", point->expected);
		bth_field(" actual=", point->actual);
		bth_field(" uart_handler=", i == 0U ? (uintptr_t)NMI_Handler : state->uart_handler);
		bth_end();
	}
	bth_puts("zephyr_bth vectors_selftest version=2 provider=");
	bth_puts(BES_BOOT_VECTORS_PROVIDER_NAME);
	bth_puts(" checks=");
	bth_dec(checks);
	bth_puts(" pass=");
	bth_dec(passed);
	bth_puts(" fail=");
	bth_dec(checks - passed);
	bth_end();
	return passed == checks ? 0 : -1;
}
