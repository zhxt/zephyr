/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "boot_contract.h"
#include "format_vectors.h"

int boot_format_selftest(void)
{
	unsigned int passed = boot_format_vectors();

	bth_puts("zephyr_bth format_selftest version=1 provider=");
	bth_puts(BES_BOOT_FORMAT_PROVIDER_NAME);
	bth_puts(" cases=");
	bth_dec(BOOT_FORMAT_SELFTEST_CASES);
	bth_puts(" pass=");
	bth_dec(passed);
	bth_puts(" fail=");
	bth_dec(BOOT_FORMAT_SELFTEST_CASES - passed);
	bth_end();
	return passed == BOOT_FORMAT_SELFTEST_CASES ? 0 : -1;
}
