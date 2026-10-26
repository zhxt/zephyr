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

#ifndef BES_BOOT_SPI_TRANSPORT_PUBLIC_H_
#define BES_BOOT_SPI_TRANSPORT_PUBLIC_H_

#include <stdint.h>

#define BOOT_SPI_PUBLIC_CR0 0x00U
#define BOOT_SPI_PUBLIC_CR1 0x04U
#define BOOT_SPI_PUBLIC_DATA 0x08U
#define BOOT_SPI_PUBLIC_STATUS 0x0cU
#define BOOT_SPI_PUBLIC_PRESCALE 0x10U
#define BOOT_SPI_PUBLIC_CLEAR 0x20U
#define BOOT_SPI_PUBLIC_DMA 0x24U
#define BOOT_SPI_PUBLIC_RX_CONTROL 0x88U
#define BOOT_SPI_PUBLIC_DSS_MASK (31U << 16)
#define BOOT_SPI_PUBLIC_DSS_27 (26U << 16)
#define BOOT_SPI_PUBLIC_SSE (1U << 1)
#define BOOT_SPI_PUBLIC_SLAVE (1U << 2)
#define BOOT_SPI_PUBLIC_LOOPBACK (1U << 0)
#define BOOT_SPI_PUBLIC_BUSY (1U << 4)
#define BOOT_SPI_PUBLIC_RX_READY (1U << 2)
#define BOOT_SPI_PUBLIC_TX_READY (1U << 1)
#define BOOT_SPI_PUBLIC_CS_MASK (15U << 7)
#define BOOT_SPI_PUBLIC_CS_SHIFT 7U

/* Platform-provided MMIO hooks; base addresses are caller-supplied data. */
uint32_t boot_spi_read32(uintptr_t address);
void boot_spi_write32(uintptr_t address, uint32_t value);

void boot_spi_public_set_transfer(uintptr_t base, uint32_t cr0, uint32_t rx_control);
void boot_spi_public_enable(uintptr_t base, uint32_t cr1, uint32_t prescale, uint32_t dma);
void boot_spi_public_select_cs(uintptr_t base, uint32_t cs);

/* Serialized master operation, one non-NULL four-byte frame, no DMA. */
int boot_spi_public_send(uintptr_t base, const void *data);
int boot_spi_public_recv(uintptr_t base, const void *command, void *data);

#endif /* BES_BOOT_SPI_TRANSPORT_PUBLIC_H_ */
