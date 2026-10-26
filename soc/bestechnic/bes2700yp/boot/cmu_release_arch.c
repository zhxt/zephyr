/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cmu_release_native.h"
#include "arch.h"

__attribute__((section(".boot_text_flash.boot_cmu_release_sync"), noinline))
void boot_cmu_release_sync(void)
{
	__DSB();
}

__attribute__((section(".boot_text_flash.boot_cmu_release_wait"), noinline))
void boot_cmu_release_wait(void)
{
	for (;;) {
		__WFI();
	}
}
