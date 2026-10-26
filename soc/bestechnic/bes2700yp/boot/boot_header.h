/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_HEADER_H_
#define BES_BOOT_HEADER_H_

#include <stddef.h>
#include <stdint.h>

/* Preserve the observed nonsecure pre-download format as complete words. */
struct bes_boot_header {
	uint32_t magic;
	uint32_t format;
	uint32_t reserved;
	const char *build_info;
};

_Static_assert(sizeof(void *) == 4, "The boot format requires 32-bit addresses");
_Static_assert(sizeof(struct bes_boot_header) == 16, "Unexpected boot header size");
_Static_assert(_Alignof(struct bes_boot_header) == 4, "Unexpected boot header alignment");
_Static_assert(offsetof(struct bes_boot_header, build_info) == 12,
	       "Unexpected build info address offset");

extern const char sys_build_info[];
extern const struct bes_boot_header boot_struct;
extern uint32_t reboot_param[4];

#endif /* BES_BOOT_HEADER_H_ */
