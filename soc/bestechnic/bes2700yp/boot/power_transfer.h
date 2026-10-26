/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_POWER_TRANSFER_H_
#define BOOT_POWER_TRANSFER_H_

#include "analog_native.h"
#include "power_leaf_native.h"

__attribute__((always_inline)) static inline uint16_t boot_power_read16(uint32_t address)
{
	uint16_t value = 0U;
	int result = hal_analogif_reg_read(address, &value);

	if (result != 0) {
		boot_power_leaf_fail(0U, address, result);
	}
	return value;
}

__attribute__((always_inline)) static inline void boot_power_write16(uint32_t address,
								   uint16_t value)
{
	int result = hal_analogif_reg_write(address, value);

	if (result != 0) {
		boot_power_leaf_fail(1U, address, result);
	}
}

#endif /* BOOT_POWER_TRANSFER_H_ */
