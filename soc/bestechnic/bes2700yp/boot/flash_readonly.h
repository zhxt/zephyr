/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef BES2700YP_BOOT_FLASH_READONLY_H_
#define BES2700YP_BOOT_FLASH_READONLY_H_

#include <stdint.h>

int boot_flash_readonly_prepare(uint32_t controller);
int boot_flash_readonly_finish(uint32_t actual_crc, uint32_t expected_crc, uint32_t bytes);
void boot_flash_readonly_dump(void);

#endif /* BES2700YP_BOOT_FLASH_READONLY_H_ */
