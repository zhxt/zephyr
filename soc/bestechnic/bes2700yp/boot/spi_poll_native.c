/* SPDX-License-Identifier: Apache-2.0 */

#include "spi_poll_native.h"
#include "spi_transport_public.h"

#define NORMAL(name) __attribute__((section(".text." #name), noinline))

static int check_frame(uint32_t id, uint32_t cr0, const void *command,
		       const void *data, uint32_t length)
{
	if (id != 0U) {
		boot_spi_control_fail(BOOT_SPI_TRANSPORT_CONTROLLER, id);
	}
	if ((cr0 & BOOT_SPI_PUBLIC_DSS_MASK) != BOOT_SPI_PUBLIC_DSS_27) {
		boot_spi_control_fail(BOOT_SPI_TRANSPORT_FRAME, cr0);
	}
	if (length == 0U || (length & 3U) != 0U) {
		return -1;
	}
	if (length != sizeof(uint32_t)) {
		boot_spi_control_fail(BOOT_SPI_TRANSPORT_LENGTH, length);
	}
	if (command == NULL || data == NULL) {
		boot_spi_control_fail(BOOT_SPI_TRANSPORT_BUFFER, 0U);
	}
	return 0;
}

NORMAL(boot_spi_poll_send)
int boot_spi_poll_send(uint32_t id, uint32_t cr0, uint32_t dma,
		       const void *data, uint32_t length)
{
	if ((dma & 2U) != 0U) {
		boot_spi_control_fail(13U, id);
	}
	int result = check_frame(id, cr0, data, data, length);

	if (result != 0) {
		return result;
	}
	return boot_spi_public_send(BOOT_SPI_SELECTED_BASE, data);
}

NORMAL(boot_spi_poll_recv)
int boot_spi_poll_recv(uint32_t id, uint32_t cr0, uint32_t dma,
		       const void *command, void *data, uint32_t length)
{
	if ((dma & 3U) != 0U) {
		boot_spi_control_fail(14U, id);
	}
	int result = check_frame(id, cr0, command, data, length);

	if (result != 0) {
		return result;
	}
	return boot_spi_public_recv(BOOT_SPI_SELECTED_BASE, command, data);
}

NORMAL(hal_ispi_send)
int hal_ispi_send(const void *data, uint32_t length)
{
	return boot_spi_poll_send(0U, boot_spi_ispi_control.tx_cr0,
				  boot_spi_ispi_control.dma, data, length);
}

NORMAL(hal_ispi_recv)
int hal_ispi_recv(const void *command, void *data, uint32_t length)
{
	return boot_spi_poll_recv(0U, boot_spi_ispi_control.rx_cr0,
				  boot_spi_ispi_control.dma, command, data, length);
}
