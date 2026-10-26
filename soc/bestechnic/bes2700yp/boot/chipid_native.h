/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_CHIPID_NATIVE_H_
#define BES_BOOT_CHIPID_NATIVE_H_

#include <stdint.h>

/* One-byte revision state in the selected loader ABI. */
extern uint8_t metal_id;
void hal_chipid_init(void);

/* Hardware acquisition and publication remain explicit backend operations. */
uint32_t read_hw_metal_id(void);
void save_hw_metal_id(uint32_t revision);

#endif /* BES_BOOT_CHIPID_NATIVE_H_ */
