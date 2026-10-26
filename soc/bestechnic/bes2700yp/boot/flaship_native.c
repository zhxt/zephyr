/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/sys/util.h>

#include "flaship_native.h"
#include "main_cmu_frequency_native.h"

#define SRAM(name) __attribute__((section(".boot_text_sram." #name), noinline))

/* Configuration and FIFO access must not fetch these addresses from XIP. */
__attribute__((section(".boot_data.flaship_bases"))) static const uintptr_t flaship_bases[2] = {
	0x40140000U, 0x50190000U};

#define CONTROL    0x04U
#define TX_FIFO    0x08U
#define STATUS     0x0cU
#define RX_FIFO    0x10U
#define CONFIG     0x14U
#define CLEAR      0x18U
#define BLOCK_MASK 0x01fff000U

__attribute__((always_inline)) static inline uint32_t read32(uintptr_t address)
{
	return boot_main_cmu_frequency_read(address);
}

__attribute__((always_inline)) static inline void write32(uintptr_t address, uint32_t value)
{
	boot_main_cmu_frequency_write(address, value);
}

__attribute__((always_inline)) static inline void idle(uintptr_t base)
{
	while ((read32(base + STATUS) & 1U) != 0U) {
	}
}

__attribute__((always_inline)) static inline void update(uintptr_t address, uint32_t mask,
							 uint32_t value)
{
	write32(address, (read32(address) & ~mask) | value);
}

SRAM(norflaship_continuous_read_off)
void norflaship_continuous_read_off(uint32_t id)
{
	uintptr_t base = flaship_bases[id];

	idle(base);
	update(base + CONTROL, BIT(25), 0U);
}

SRAM(norflaship_write_txfifo)
void norflaship_write_txfifo(uint32_t id, const uint8_t *buffer, uint32_t length)
{
	uintptr_t base = flaship_bases[id];

	update(base + CONTROL, BLOCK_MASK, (length << 12) & BLOCK_MASK);
	for (;;) {
		uint32_t available = (read32(base + STATUS) >> 8) & 31U;
		uint32_t count = MIN(available, length);

		/* Retain the original retry's prefix; do not invent FIFO progress. */
		for (uint32_t i = 0U; i < count; i++) {
			boot_flaship_write8(base + TX_FIFO, buffer[i]);
		}
		if (count == length) {
			return;
		}
	}
}

SRAM(norflaship_read_rxfifo)
uint8_t norflaship_read_rxfifo(uint32_t id)
{
	return (uint8_t)read32(flaship_bases[id] + RX_FIFO);
}

SRAM(norflaship_cmd_addr)
void norflaship_cmd_addr(uint32_t id, uint32_t command, uint32_t address)
{
	uintptr_t base = flaship_bases[id];

	idle(base);
	update(base + CONTROL, 1U, 0U);
	write32(base, command | (address << 8));
}

SRAM(norflaship_ext_tx_cmd)
void norflaship_ext_tx_cmd(uint32_t id, uint32_t command, uint32_t length)
{
	uintptr_t base = flaship_bases[id];

	idle(base);
	update(base + CONTROL, BLOCK_MASK, ((length << 12) & BLOCK_MASK) | 1U);
	write32(base, command);
}

SRAM(norflaship_ext_rx_cmd)
void norflaship_ext_rx_cmd(uint32_t id, uint32_t command, uint32_t tx_length, uint32_t rx_length)
{
	uintptr_t base = flaship_bases[id];

	idle(base);
	update(base + CONTROL, BLOCK_MASK, ((tx_length << 12) & BLOCK_MASK) | 1U);
	write32(base, command | ((rx_length << 13) & 0x03ffe000U) | 0x100U);
}

SRAM(norflaship_cmd_done)
void norflaship_cmd_done(uint32_t id)
{
	uintptr_t base = flaship_bases[id];

	idle(base);
	update(base + CONTROL, 1U, 0U);
}

SRAM(norflaship_rxfifo_count_wait)
void norflaship_rxfifo_count_wait(uint32_t id, uint32_t count)
{
	while (((read32(flaship_bases[id] + STATUS) >> 4) & 15U) < count) {
	}
}

SRAM(norflaship_busy_wait)
void norflaship_busy_wait(uint32_t id)
{
	idle(flaship_bases[id]);
}

SRAM(norflaship_is_busy)
uint32_t norflaship_is_busy(uint32_t id)
{
	return read32(flaship_bases[id] + STATUS) & 1U;
}

__attribute__((always_inline)) static inline void clear(uint32_t id, uint32_t mask)
{
	uintptr_t base = flaship_bases[id];

	idle(base);
	write32(base + CLEAR, mask);
	idle(base);
}

SRAM(norflaship_clear_fifos)
void norflaship_clear_fifos(uint32_t id)
{
	clear(id, 3U);
}

SRAM(norflaship_clear_rxfifo)
void norflaship_clear_rxfifo(uint32_t id)
{
	clear(id, 1U);
}

SRAM(norflaship_div)
void norflaship_div(uint32_t id, uint32_t value)
{
	uintptr_t base = flaship_bases[id];

	idle(base);
	update(base + CONFIG, 0xff0000U, (value << 16) & 0xff0000U);
}

SRAM(norflaship_get_div)
uint32_t norflaship_get_div(uint32_t id)
{
	return (read32(flaship_bases[id] + CONFIG) >> 16) & 0xffU;
}

SRAM(norflaship_samdly)
void norflaship_samdly(uint32_t id, uint32_t value)
{
	uintptr_t base = flaship_bases[id];

	idle(base);
	update(base + CONFIG, 0xf000U, (value << 12) & 0xf000U);
}

__attribute__((always_inline)) static inline void mode(uint32_t id, uint32_t offset, uint32_t mask,
						       uint32_t value, uint32_t wait)
{
	uintptr_t base = flaship_bases[id];

	if (wait != 0U) {
		idle(base);
	}
	update(base + offset, mask, value != 0U ? mask : 0U);
}

SRAM(norflaship_dual_mode)
void norflaship_dual_mode(uint32_t id, uint32_t value)
{
	mode(id, CONFIG, BIT(5), value, 1U);
}

SRAM(norflaship_quad_mode)
void norflaship_quad_mode(uint32_t id, uint32_t value)
{
	mode(id, CONFIG, BIT(4), value, 1U);
}

SRAM(norflaship_4byteaddr_mode)
void norflaship_4byteaddr_mode(uint32_t id, uint32_t value)
{
	mode(id, CONFIG, BIT(3), value, 0U);
}

SRAM(norflaship_nand_sel)
void norflaship_nand_sel(uint32_t id, uint32_t value)
{
	mode(id, 0x1cU, BIT(0), value, 0U);
}

SRAM(norflaship_rdcmd)
void norflaship_rdcmd(uint32_t id, uint32_t value)
{
	update(flaship_bases[id] + 0x20U, 0xff0000U, (value << 16) & 0xff0000U);
}

SRAM(norflaship_qrdcmd)
void norflaship_qrdcmd(uint32_t id, uint32_t value)
{
	update(flaship_bases[id] + 0x20U, 0xffU, value & 0xffU);
}

SRAM(norflaship_set_idle_io_dir)
void norflaship_set_idle_io_dir(uint32_t id, uint32_t value)
{
	update(flaship_bases[id] + 0x34U, 0x3c0000U, (value << 18) & 0x3c0000U);
}

SRAM(norflaship_dtr_mode)
void norflaship_dtr_mode(uint32_t id, uint32_t value)
{
	mode(id, 0x40U, 0x0cU, value, 1U);
}

SRAM(norflaship_mem_read_bus_lock)
uint32_t norflaship_mem_read_bus_lock(uint32_t id)
{
	uintptr_t base = flaship_bases[id];
	uint32_t key = read32(base + 0x34U) & BIT(8);

	if (key == 0U) {
		/* Acquire using a fresh register sample, not the key's old word. */
		update(base + 0x34U, BIT(8), BIT(8));
		idle(base);
	}
	return key;
}

SRAM(norflaship_mem_read_bus_unlock)
void norflaship_mem_read_bus_unlock(uint32_t id, uint32_t key)
{
	if (key == 0U) {
		uintptr_t base = flaship_bases[id];

		idle(base);
		update(base + 0x34U, BIT(8), 0U);
	}
}
