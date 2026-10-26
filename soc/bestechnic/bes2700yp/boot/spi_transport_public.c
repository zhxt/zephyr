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
 * Selected master transport adapted from OpenHarmony device_soc_bestechnic
 * revision 17f61a22388e6f2fbd8fe855b0ccb298b9a3bd45, hal_spi.c and reg_spi.h.
 * See spi_transport_public.rst for the source and adaptation boundary.
 */

#include <zephyr/sys/byteorder.h>

#include "spi_transport_public.h"

void boot_spi_public_set_transfer(uintptr_t base, uint32_t cr0, uint32_t rx_control)
{
	boot_spi_write32(base + BOOT_SPI_PUBLIC_CR0, cr0);
	boot_spi_write32(base + BOOT_SPI_PUBLIC_RX_CONTROL, rx_control);
}

void boot_spi_public_enable(uintptr_t base, uint32_t cr1, uint32_t prescale, uint32_t dma)
{
	boot_spi_write32(base + BOOT_SPI_PUBLIC_CR1, cr1);
	boot_spi_write32(base + BOOT_SPI_PUBLIC_PRESCALE, prescale);
	boot_spi_write32(base + BOOT_SPI_PUBLIC_DMA, dma);
}

void boot_spi_public_select_cs(uintptr_t base, uint32_t cs)
{
	uint32_t cr1 = boot_spi_read32(base + BOOT_SPI_PUBLIC_CR1);

	boot_spi_write32(base + BOOT_SPI_PUBLIC_CR1,
			 (cr1 & ~BOOT_SPI_PUBLIC_CS_MASK) | (cs << BOOT_SPI_PUBLIC_CS_SHIFT));
}

static void wait_idle(uintptr_t base)
{
	while ((boot_spi_read32(base + BOOT_SPI_PUBLIC_CR1) & BOOT_SPI_PUBLIC_SSE) != 0U &&
	       (boot_spi_read32(base + BOOT_SPI_PUBLIC_STATUS) & BOOT_SPI_PUBLIC_BUSY) != 0U) {
	}
}

int boot_spi_public_send(uintptr_t base, const void *data)
{
	uint32_t sent = 0U;

	while (sent < sizeof(uint32_t)) {
		if ((boot_spi_read32(base + BOOT_SPI_PUBLIC_CR1) & BOOT_SPI_PUBLIC_SSE) == 0U) {
			break;
		}
		if ((boot_spi_read32(base + BOOT_SPI_PUBLIC_STATUS) &
		     BOOT_SPI_PUBLIC_TX_READY) != 0U) {
			boot_spi_write32(base + BOOT_SPI_PUBLIC_DATA, sys_get_le32(data));
			sent += sizeof(uint32_t);
		}
	}
	int result = sent < sizeof(uint32_t) ? 1 : 0;

	wait_idle(base);
	return result;
}

int boot_spi_public_recv(uintptr_t base, const void *command, void *data)
{
	uint32_t sent = 0U;
	uint32_t received = 0U;

	if ((boot_spi_read32(base + BOOT_SPI_PUBLIC_STATUS) & BOOT_SPI_PUBLIC_BUSY) != 0U) {
		return -11;
	}
	while ((boot_spi_read32(base + BOOT_SPI_PUBLIC_STATUS) & BOOT_SPI_PUBLIC_RX_READY) != 0U) {
		(void)boot_spi_read32(base + BOOT_SPI_PUBLIC_DATA);
	}
	boot_spi_write32(base + BOOT_SPI_PUBLIC_CLEAR, UINT32_MAX);
	while (sent < sizeof(uint32_t) || received < sizeof(uint32_t)) {
		if ((boot_spi_read32(base + BOOT_SPI_PUBLIC_CR1) & BOOT_SPI_PUBLIC_SSE) == 0U) {
			break;
		}
		if (sent < sizeof(uint32_t) &&
		    (boot_spi_read32(base + BOOT_SPI_PUBLIC_STATUS) &
		     BOOT_SPI_PUBLIC_TX_READY) != 0U) {
			boot_spi_write32(base + BOOT_SPI_PUBLIC_DATA, sys_get_le32(command));
			sent += sizeof(uint32_t);
		}
		if (received < sizeof(uint32_t) &&
		    (boot_spi_read32(base + BOOT_SPI_PUBLIC_STATUS) &
		     BOOT_SPI_PUBLIC_RX_READY) != 0U) {
			sys_put_le32(boot_spi_read32(base + BOOT_SPI_PUBLIC_DATA), data);
			received += sizeof(uint32_t);
		}
	}
	int result = sent < sizeof(uint32_t) || received < sizeof(uint32_t) ? 1 : 0;

	wait_idle(base);
	return result;
}
