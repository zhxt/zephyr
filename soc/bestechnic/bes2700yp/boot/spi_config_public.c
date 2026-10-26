/*
 * Copyright (c) 2021 Bestechnic (Shanghai) Co., Ltd. All rights reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 Bestechnic (Shanghai) Co., Ltd.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Divider calculation and register fields adapted from OpenHarmony
 * device_soc_bestechnic, revision 17f61a22388e6f2fbd8fe855b0ccb298b9a3bd45,
 * platform/hal/hal_spi.c and platform/hal/reg_spi.h. See spi_config_public.rst.
 */

#include <stddef.h>

#include "spi_config_public.h"

#define SPI_SCR_MAX 255U
#define SPI_PRESCALE_MAX 254U
#define SPI_DSS_SHIFT 16U
#define SPI_SCR_SHIFT 8U
#define SPI_PHASE (1U << 7)
#define SPI_POLARITY (1U << 6)
#define SPI_RX_SELECT (1U << 11)
#define SPI_CS_SHIFT 7U
#define SPI_ENABLE (1U << 1)
#define SPI_RX_FRAME_ENABLE (1U << 6)
#define SPI_RX_FRAME_POLARITY (1U << 5)

static uint32_t divide_round_up(uint32_t value, uint32_t divisor)
{
	return value / divisor + (value % divisor != 0U ? 1U : 0U);
}

enum boot_spi_public_status boot_spi_config_compute(const struct boot_spi_public_config *config,
						   uint32_t source_hz,
						   struct boot_spi_public_words *words)
{
	uint32_t clock = source_hz;
	uint32_t selector = 1U;
	uint32_t divisor;
	uint32_t prescale;
	uint32_t serial;
	uint32_t common;

	if (config == NULL || words == NULL) {
		return BOOT_SPI_PUBLIC_ARGUMENT;
	}
	if (source_hz < 2U || source_hz > UINT32_MAX / 2U) {
		return BOOT_SPI_PUBLIC_CLOCK;
	}
	if (config->rate > source_hz / 2U) {
		clock *= 2U;
		selector = 2U;
	}
	if (config->rate > clock / 2U) {
		return BOOT_SPI_PUBLIC_RATE_HIGH;
	}
	if (config->rate == 0U || config->rate < clock / (SPI_PRESCALE_MAX * 256U)) {
		return BOOT_SPI_PUBLIC_RATE_LOW;
	}
	if (config->tx_bits < 4U || config->tx_bits > 32U) {
		return BOOT_SPI_PUBLIC_TX_BITS;
	}
	if (config->rx_bits < 4U || config->rx_bits > 32U) {
		return BOOT_SPI_PUBLIC_RX_BITS;
	}
	if (config->rx_frame_bits > 32U ||
	    (config->rx_frame_bits != 0U && config->rx_bits >= config->rx_frame_bits)) {
		return BOOT_SPI_PUBLIC_RX_FRAME;
	}
	if (config->cs > 3U) {
		return BOOT_SPI_PUBLIC_CS;
	}
	divisor = divide_round_up(clock, config->rate);
	prescale = divide_round_up(divisor, SPI_SCR_MAX + 1U);
	if (prescale < 2U) {
		prescale = 2U;
	} else {
		prescale += prescale & 1U;
		if (prescale > SPI_PRESCALE_MAX) {
			prescale = SPI_PRESCALE_MAX;
		}
	}
	serial = divide_round_up(divisor, prescale) - 1U;
	if (serial > SPI_SCR_MAX) {
		serial = SPI_SCR_MAX;
	}
	common = (serial << SPI_SCR_SHIFT) | (config->phase ? SPI_PHASE : 0U) |
		 (config->polarity ? SPI_POLARITY : 0U);
	words->clock_hz = clock;
	words->clock_select = (uint8_t)selector;
	words->period = prescale * (serial + 1U) - 1U;
	words->tx_cr0 = common | ((uint32_t)(config->tx_bits - 1U) << SPI_DSS_SHIFT);
	words->rx_cr0 = common | ((uint32_t)((config->rx_frame_bits != 0U ?
		config->rx_frame_bits : config->rx_bits) - 1U) << SPI_DSS_SHIFT);
	words->cr1 = (uint16_t)((config->separate_receive ? SPI_RX_SELECT : 0U) |
		((uint32_t)config->cs << SPI_CS_SHIFT) | BOOT_SPI_PUBLIC_SOD | SPI_ENABLE);
	words->prescale = (uint16_t)prescale;
	words->dma = (uint8_t)((config->dma_tx ? 2U : 0U) | (config->dma_rx ? 1U : 0U));
	words->tx_control2 = 0U;
	words->rx_control2 = config->rx_frame_bits != 0U ?
		(uint8_t)(SPI_RX_FRAME_ENABLE | SPI_RX_FRAME_POLARITY |
			  (config->rx_bits - 1U)) : 0U;
	return BOOT_SPI_PUBLIC_OK;
}
