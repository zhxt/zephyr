/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_SPI_POLL_NATIVE_H_
#define BOOT_SPI_POLL_NATIVE_H_

#include "spi_control_native.h"

/* Fixed loader ABI: controller zero, 27 bits, one four-byte non-NULL frame.
 * Accessible ordinary RAM and serialized calls; hardware controls completion.
 */
int boot_spi_poll_send(uint32_t id, uint32_t cr0, uint32_t dma,
		       const void *data, uint32_t length);
int boot_spi_poll_recv(uint32_t id, uint32_t cr0, uint32_t dma,
		       const void *command, void *data, uint32_t length);
int hal_ispi_send(const void *data, uint32_t length);
int hal_ispi_recv(const void *command, void *data, uint32_t length);

#endif /* BOOT_SPI_POLL_NATIVE_H_ */
