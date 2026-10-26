/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_ANALOG_NATIVE_H_
#define BOOT_ANALOG_NATIVE_H_

#include "spi_poll_native.h"

/* Fixed loader register-word ABI and serialized ordinary-RAM output. */
int hal_analogif_reg_read(uint32_t address, uint16_t *value);
int hal_analogif_reg_write(uint32_t address, uint32_t value);
int hal_analogif_open(void);

extern uint8_t boot_analog_cs;
extern uint8_t boot_analog_revision;
extern struct boot_spi_control boot_analog_control;
extern struct boot_spi_control boot_analog_internal_control;

#endif /* BOOT_ANALOG_NATIVE_H_ */
