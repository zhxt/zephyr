/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_BOOTMODE_NATIVE_H_
#define BES_BOOT_BOOTMODE_NATIVE_H_

#include <stdint.h>

/* Opaque one-byte result matching the fixed loader's boot-mode return ABI. */
union boot_hw_bootmode {
	uint8_t reg;
};

void hal_hw_bootmode_init(void);
union boot_hw_bootmode hal_hw_bootmode_get(void);

/* Capture yields the normalized byte without acknowledging it. A backend must
 * supply the actual hardware acknowledgment protocol separately.
 */
uint8_t boot_mode_hw_capture(void);
void boot_mode_hw_acknowledge(void);

#endif /* BES_BOOT_BOOTMODE_NATIVE_H_ */
