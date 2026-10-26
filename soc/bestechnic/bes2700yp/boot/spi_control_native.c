/* SPDX-License-Identifier: Apache-2.0 */

#include "spi_control_native.h"
#include "spi_transport_public.h"
#include "cmu_gate_native.h"
#include "cmu_reset_native.h"
#include "cmu_periph_native.h"
#include "main_cmu_periph_native.h"

#define EARLY(name) __attribute__((section(".boot_bss." #name)))
#define NORMAL(name) __attribute__((section(".text." #name), noinline))

struct boot_spi_control boot_spi_ispi_control EARLY(boot_spi_ispi_control);
uint8_t boot_spi_cs_map[3] EARLY(boot_spi_cs_map);
uint8_t boot_spi_tx_dma_channel[3] EARLY(boot_spi_tx_dma_channel);
uint8_t boot_spi_rx_dma_channel[3] EARLY(boot_spi_rx_dma_channel);
uint8_t boot_spi_clock_select[3] EARLY(boot_spi_clock_select);
uint8_t boot_spi_initialized EARLY(boot_spi_initialized);

/* Retained target clock/reset selectors; their hardware source review is separate. */
static const uint8_t selected_modules[2] = {35U, 24U};

static void check_control(uint32_t id, const struct boot_spi_control *control, uint32_t receive)
{
	if (id != 0U) {
		boot_spi_control_fail(BOOT_SPI_TRANSPORT_CONTROLLER, id);
	}
	if (receive != 0U) {
		boot_spi_control_fail(BOOT_SPI_TRANSPORT_RECEIVE_SETUP, receive);
	}
	if ((control->tx_cr0 & BOOT_SPI_PUBLIC_DSS_MASK) != BOOT_SPI_PUBLIC_DSS_27 ||
	    control->rx_cr0 != control->tx_cr0) {
		boot_spi_control_fail(BOOT_SPI_TRANSPORT_FRAME, control->tx_cr0);
	}
	if (control->dma != 0U || control->tx_control2 != 0U || control->rx_control2 != 0U ||
	    control->sample_delay != 0U ||
	    (control->cr1 & (BOOT_SPI_PUBLIC_SLAVE | BOOT_SPI_PUBLIC_LOOPBACK)) != 0U) {
		boot_spi_control_fail(BOOT_SPI_TRANSPORT_MODE, control->cr1);
	}
	if (control->clock_select != 1U) {
		boot_spi_control_fail(BOOT_SPI_TRANSPORT_CLOCK, control->clock_select);
	}
}

NORMAL(boot_spi_set_transfer_type)
void boot_spi_set_transfer_type(uint32_t id, const struct boot_spi_control *control,
				uint32_t receive)
{
	check_control(id, control, receive);
	boot_spi_public_set_transfer(BOOT_SPI_SELECTED_BASE, control->tx_cr0, control->tx_control2);
}

NORMAL(boot_spi_enable)
void boot_spi_enable(uint32_t id, const struct boot_spi_control *control, uint32_t receive)
{
	uint32_t clock;

	boot_spi_set_transfer_type(id, control, receive);
	boot_spi_public_enable(BOOT_SPI_SELECTED_BASE, control->cr1,
			       control->prescale, control->dma);
	clock = control->clock_select;
	if (clock == boot_spi_clock_select[id]) {
		return;
	}
	boot_spi_clock_select[id] = (uint8_t)clock;
	(void)hal_cmu_ispi_set_freq(0U);
}

NORMAL(boot_spi_open)
int boot_spi_open(uint32_t id, const struct boot_spi_config *config,
		  struct boot_spi_control *control)
{
	if (id != 0U) {
		boot_spi_control_fail(BOOT_SPI_TRANSPORT_CONTROLLER, id);
	}
	int result = hal_spi_init_ctrl(config, control);

	if (result != 0) {
		return result;
	}
	check_control(id, control, 0U);
	if (boot_spi_initialized == 0U) {
		boot_spi_initialized = 1U;
		for (uint32_t i = 0U; i < 3U; i++) {
			boot_spi_tx_dma_channel[i] = 0xffU;
		}
		for (uint32_t i = 0U; i < 3U; i++) {
			boot_spi_rx_dma_channel[i] = 0xffU;
		}
	}
	if (boot_spi_cs_map[id] == 0U) {
		boot_spi_cs_map[id] = (uint8_t)(1U << config->cs);
		(void)hal_cmu_clock_enable(selected_modules[0]);
		(void)hal_cmu_clock_enable(selected_modules[1]);
		(void)hal_cmu_reset_clear(selected_modules[0]);
		(void)hal_cmu_reset_clear(selected_modules[1]);
	} else {
		boot_spi_cs_map[id] |= (uint8_t)(1U << config->cs);
	}
	boot_spi_enable(id, control, 0U);
	return result;
}

__attribute__((section(".boot_text_flash.hal_ispi_open"), noinline))
int hal_ispi_open(const struct boot_spi_config *config)
{
	if (config->tx_bits != config->rx_bits || config->rx_frame_bits != 0U) {
		boot_spi_control_fail(11U, 0U);
	}
	return boot_spi_open(0U, config, &boot_spi_ispi_control);
}

NORMAL(hal_ispi_enable_ctrl)
void hal_ispi_enable_ctrl(const struct boot_spi_control *control)
{
	boot_spi_enable(0U, control, 0U);
}

NORMAL(hal_ispi_activate_cs)
void hal_ispi_activate_cs(uint32_t cs)
{
	if (cs > 14U) {
		boot_spi_control_fail(12U, cs);
	}
	boot_spi_public_select_cs(BOOT_SPI_SELECTED_BASE, cs);
}
