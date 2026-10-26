/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "spi_config_native.h"
#include "spi_config_public.h"

#define BOOT_SPI_SOURCE_HZ 24000000U
#define BOOT_SPI_UNSUPPORTED_FLAGS 0xc0U

__attribute__((section(".text.hal_spi_init_ctrl"), noinline))
int hal_spi_init_ctrl(const struct boot_spi_config *config, struct boot_spi_control *control)
{
	uint32_t flags = config->flags;
	uint32_t clock = (config->rate << 1) > BOOT_SPI_SOURCE_HZ ?
			 BOOT_SPI_SOURCE_HZ * 2U : BOOT_SPI_SOURCE_HZ;
	struct boot_spi_public_config input = {
		.rate = config->rate,
		.tx_bits = config->tx_bits,
		.rx_bits = config->rx_bits,
		.rx_frame_bits = config->rx_frame_bits,
		.cs = config->cs,
		.phase = (flags & 1U) != 0U,
		.polarity = (flags & 2U) != 0U,
		.separate_receive = (flags & 0x20U) != 0U,
		.dma_rx = (flags & 8U) != 0U,
		.dma_tx = (flags & 0x10U) != 0U,
	};
	struct boot_spi_public_words words;
	enum boot_spi_public_status status;

	if ((flags & 4U) != 0U) {
		boot_spi_config_fail(BOOT_SPI_CONFIG_SLAVE, 0U, 0U);
	}
	if ((flags & BOOT_SPI_UNSUPPORTED_FLAGS) != 0U) {
		boot_spi_config_fail(BOOT_SPI_CONFIG_UNSUPPORTED,
				     flags & BOOT_SPI_UNSUPPORTED_FLAGS, 0U);
	}
	control->clock_select = clock == BOOT_SPI_SOURCE_HZ ? 1U : 2U;
	status = boot_spi_config_compute(&input, BOOT_SPI_SOURCE_HZ, &words);
	switch (status) {
	case BOOT_SPI_PUBLIC_OK:
		break;
	case BOOT_SPI_PUBLIC_RATE_HIGH:
		boot_spi_config_fail(BOOT_SPI_CONFIG_RATE_HIGH, config->rate, clock / 2U);
	case BOOT_SPI_PUBLIC_RATE_LOW:
		boot_spi_config_fail(BOOT_SPI_CONFIG_RATE_LOW, config->rate, clock / (254U * 256U));
	case BOOT_SPI_PUBLIC_TX_BITS:
		boot_spi_config_fail(BOOT_SPI_CONFIG_TX_BITS, config->tx_bits, 0U);
	case BOOT_SPI_PUBLIC_RX_BITS:
		boot_spi_config_fail(BOOT_SPI_CONFIG_RX_BITS, config->rx_bits, 0U);
	case BOOT_SPI_PUBLIC_RX_FRAME:
		boot_spi_config_fail(BOOT_SPI_CONFIG_RX_FRAME, config->rx_frame_bits, 0U);
	case BOOT_SPI_PUBLIC_CS:
		boot_spi_config_fail(BOOT_SPI_CONFIG_CS, config->cs, 0U);
	default:
		boot_spi_config_fail(BOOT_SPI_CONFIG_UNSUPPORTED, (uint32_t)status, 5U);
	}
	if (config->tx_bits == 32U) {
		boot_spi_config_fail(BOOT_SPI_CONFIG_UNSUPPORTED, 32U, 1U);
	}
	if (config->rx_bits == 32U) {
		boot_spi_config_fail(BOOT_SPI_CONFIG_UNSUPPORTED, 32U, 2U);
	}
	if (config->rx_frame_bits == 32U) {
		boot_spi_config_fail(BOOT_SPI_CONFIG_UNSUPPORTED, 32U, 3U);
	}
	control->sample_max = (uint8_t)words.period;
	if (words.period > UINT8_MAX) {
		boot_spi_config_fail(BOOT_SPI_CONFIG_SAMPLE_MAX, words.period, 0U);
	}
	if (config->sample_delay > words.period) {
		boot_spi_config_fail(BOOT_SPI_CONFIG_SAMPLE_DELAY, config->sample_delay,
				     words.period);
	}
	if (config->sample_delay != 0U) {
		boot_spi_config_fail(BOOT_SPI_CONFIG_UNSUPPORTED, config->sample_delay, 4U);
	}
	control->tx_cr0 = words.tx_cr0;
	control->rx_cr0 = words.rx_cr0;
	/* Preserve the selected loader's constructed word, including clear SOD. */
	control->cr1 = words.cr1 & (uint16_t)~BOOT_SPI_PUBLIC_SOD;
	control->prescale = (uint8_t)words.prescale;
	control->dma = words.dma;
	control->tx_control2 = words.tx_control2;
	control->rx_control2 = words.rx_control2;
	control->sample_delay = 0U;
	control->clock_select = words.clock_select;
	return 0;
}
