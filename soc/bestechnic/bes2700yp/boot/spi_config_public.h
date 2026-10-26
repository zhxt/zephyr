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
 */

#ifndef BES_BOOT_SPI_CONFIG_PUBLIC_H_
#define BES_BOOT_SPI_CONFIG_PUBLIC_H_

#include <stdbool.h>
#include <stdint.h>

/* Public reference's bit-11 RX select and four-bit chip-select layout. */
#define BOOT_SPI_PUBLIC_SOD (1U << 3)

struct boot_spi_public_config {
	uint32_t rate;
	uint8_t tx_bits;
	uint8_t rx_bits;
	uint8_t rx_frame_bits;
	uint8_t cs;
	bool phase;
	bool polarity;
	bool separate_receive;
	bool dma_rx;
	bool dma_tx;
};

struct boot_spi_public_words {
	uint32_t clock_hz;
	uint32_t period;
	uint32_t tx_cr0;
	uint32_t rx_cr0;
	uint16_t cr1;
	uint16_t prescale;
	uint8_t dma;
	uint8_t tx_control2;
	uint8_t rx_control2;
	uint8_t clock_select;
};

enum boot_spi_public_status {
	BOOT_SPI_PUBLIC_OK,
	BOOT_SPI_PUBLIC_ARGUMENT,
	BOOT_SPI_PUBLIC_CLOCK,
	BOOT_SPI_PUBLIC_RATE_HIGH,
	BOOT_SPI_PUBLIC_RATE_LOW,
	BOOT_SPI_PUBLIC_TX_BITS,
	BOOT_SPI_PUBLIC_RX_BITS,
	BOOT_SPI_PUBLIC_RX_FRAME,
	BOOT_SPI_PUBLIC_CS,
};

/* Pure configuration calculation. Output fields are valid only on success. */
enum boot_spi_public_status boot_spi_config_compute(const struct boot_spi_public_config *config,
						   uint32_t source_hz,
						   struct boot_spi_public_words *words);

#endif /* BES_BOOT_SPI_CONFIG_PUBLIC_H_ */
