/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BOOT_SPI_CONFIG_NATIVE_H_
#define BOOT_SPI_CONFIG_NATIVE_H_

#include <stddef.h>
#include <stdint.h>

/* Fixed loader ABI; flags remain opaque until their bus users are qualified. */
struct boot_spi_config {
	uint32_t rate;
	uint8_t flags;
	uint8_t reserved5;
	uint8_t cs;
	uint8_t tx_bits;
	uint8_t rx_bits;
	uint8_t rx_frame_bits;
	uint8_t sample_delay;
	uint8_t reserved11;
};

struct boot_spi_control {
	uint32_t tx_cr0;
	uint32_t rx_cr0;
	uint16_t cr1;
	uint8_t prescale;
	uint8_t dma;
	uint8_t tx_control2;
	uint8_t rx_control2;
	uint8_t sample_max;
	uint8_t sample_delay;
	uint8_t clock_select;
	uint8_t reserved[3];
};

enum boot_spi_config_error {
	BOOT_SPI_CONFIG_SLAVE = 1,
	BOOT_SPI_CONFIG_RATE_HIGH,
	BOOT_SPI_CONFIG_RATE_LOW,
	BOOT_SPI_CONFIG_TX_BITS,
	BOOT_SPI_CONFIG_RX_BITS,
	BOOT_SPI_CONFIG_RX_FRAME,
	BOOT_SPI_CONFIG_RX_COMBINATION,
	BOOT_SPI_CONFIG_CS,
	BOOT_SPI_CONFIG_SAMPLE_MAX,
	BOOT_SPI_CONFIG_SAMPLE_DELAY,
	BOOT_SPI_CONFIG_UNSUPPORTED,
};

_Static_assert(sizeof(struct boot_spi_config) == 12U, "SPI config ABI size");
_Static_assert(offsetof(struct boot_spi_config, tx_bits) == 7U, "SPI config ABI");
_Static_assert(offsetof(struct boot_spi_config, sample_delay) == 10U, "SPI config ABI");
_Static_assert(sizeof(struct boot_spi_control) == 20U, "SPI control ABI size");
_Static_assert(offsetof(struct boot_spi_control, cr1) == 8U, "SPI control ABI");
_Static_assert(offsetof(struct boot_spi_control, clock_select) == 16U, "SPI control ABI");

int hal_spi_init_ctrl(const struct boot_spi_config *config, struct boot_spi_control *control);
void boot_spi_config_fail(uint32_t reason, uint32_t first, uint32_t second)
	__attribute__((noreturn));

#endif /* BOOT_SPI_CONFIG_NATIVE_H_ */
