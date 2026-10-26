/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "boot_header.h"

/* The programmer commits the valid-image magic after writing the image. */
const struct bes_boot_header boot_struct __attribute__((section(".boot_struct"), used)) = {
	.magic = UINT32_MAX,
	.format = 0x00040000,
	.reserved = 0,
	.build_info = sys_build_info,
};

/* Kept outside CRT zeroing; the linker reserves the existing 64-byte slot. */
uint32_t reboot_param[4] __attribute__((section(".reboot_param"), used)) = {0};
