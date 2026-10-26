/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_SPI_CONTROL_NATIVE_H_
#define BOOT_SPI_CONTROL_NATIVE_H_

#include "spi_config_native.h"

/* Internal, serialized loader calls; only controller zero is selected. */
enum boot_spi_transport_error {
	BOOT_SPI_TRANSPORT_CONTROLLER = 15,
	BOOT_SPI_TRANSPORT_RECEIVE_SETUP,
	BOOT_SPI_TRANSPORT_FRAME,
	BOOT_SPI_TRANSPORT_LENGTH,
	BOOT_SPI_TRANSPORT_BUFFER,
	BOOT_SPI_TRANSPORT_MODE,
	BOOT_SPI_TRANSPORT_CLOCK,
};

#define BOOT_SPI_SELECTED_BASE 0x40009000U

void boot_spi_set_transfer_type(uint32_t id, const struct boot_spi_control *control,
				uint32_t receive);
void boot_spi_enable(uint32_t id, const struct boot_spi_control *control, uint32_t receive);
int boot_spi_open(uint32_t id, const struct boot_spi_config *config,
		  struct boot_spi_control *control);
int hal_ispi_open(const struct boot_spi_config *config);
void hal_ispi_enable_ctrl(const struct boot_spi_control *control);
void hal_ispi_activate_cs(uint32_t cs);

uint32_t boot_spi_read32(uintptr_t address);
void boot_spi_write32(uintptr_t address, uint32_t value);
void boot_spi_control_fail(uint32_t reason, uint32_t argument) __attribute__((noreturn));

extern struct boot_spi_control boot_spi_ispi_control;
extern uint8_t boot_spi_cs_map[3];
extern uint8_t boot_spi_tx_dma_channel[3];
extern uint8_t boot_spi_rx_dma_channel[3];
extern uint8_t boot_spi_clock_select[3];
extern uint8_t boot_spi_initialized;

#endif /* BOOT_SPI_CONTROL_NATIVE_H_ */
