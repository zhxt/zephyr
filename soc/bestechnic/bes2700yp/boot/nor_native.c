/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/sys/util.h>

#include "flaship_native.h"
#include "nor_native.h"

#define SRAM(name) __attribute__((section(".boot_text_sram." #name), noinline))

void hal_sys_timer_delay_us(uint32_t us);

__attribute__((always_inline)) static inline uint8_t status(uint32_t id, uint32_t command)
{
	norflaship_clear_fifos(id);
	norflaship_ext_rx_cmd(id, command, 0U, 1U);
	norflaship_rxfifo_count_wait(id, 1U);
	return norflaship_read_rxfifo(id);
}

SRAM(boot_nor_set_read_div)
void boot_nor_set_read_div(uint32_t id)
{
	uint32_t divider = boot_nor_div_read[id];

	if (divider != 0U) {
		norflaship_div(id, divider);
	}
}

SRAM(norflash_read_status_s0_s7)
uint8_t norflash_read_status_s0_s7(uint32_t id)
{
	return status(id, 5U);
}

SRAM(norflash_read_status_s8_s15)
uint8_t norflash_read_status_s8_s15(uint32_t id)
{
	return status(id, 0x35U);
}

SRAM(norflash_reset)
void norflash_reset(uint32_t id)
{
	uint32_t all_ones = UINT32_MAX;

	norflaship_clear_fifos(id);
	norflaship_cmd_addr(id, 0xabU, 0U);
	hal_sys_timer_delay_us(40U);
	norflaship_4byteaddr_mode(id, 0U);
	norflaship_qrdcmd(id, 0xebU);
	norflaship_dtr_mode(id, 0U);
	norflaship_quad_mode(id, 0U);
	norflaship_dual_mode(id, 0U);
	norflaship_continuous_read_off(id);
	norflaship_busy_wait(id);
	norflaship_clear_fifos(id);
	norflaship_write_txfifo(id, (const uint8_t *)&all_ones, 4U);
	norflaship_ext_tx_cmd(id, 0xffU, 4U);
	norflaship_cmd_done(id);
	norflaship_clear_rxfifo(id);
	norflaship_ext_tx_cmd(id, 0x66U, 0U);
	norflaship_ext_tx_cmd(id, 0x99U, 0U);
	hal_sys_timer_delay_us(200U);
	norflaship_ext_tx_cmd(id, 0xe9U, 0U);
	norflaship_cmd_done(id);
	boot_nor_div_read[id] = 0U;
	boot_nor_div_others[id] = 0U;
	boot_nor_preferred_idx_adj[id] = 0U;
}

SRAM(norflash_pre_operation_suspend)
int32_t norflash_pre_operation_suspend(uint32_t id, uint32_t suspend __attribute__((unused)))
{
	boot_nor_mrb_status[id] = norflaship_mem_read_bus_lock(id);
	norflaship_busy_wait(id);
	uint32_t divider = boot_nor_div_others[id];

	if (divider != 0U) {
		norflaship_div(id, divider);
	}
	return 0;
}

SRAM(norflash_post_operation_suspend)
int32_t norflash_post_operation_suspend(uint32_t id, uint32_t suspend __attribute__((unused)),
					uint32_t resume __attribute__((unused)))
{
	norflaship_busy_wait(id);
	norflaship_cmd_done(id);
	boot_nor_set_read_div(id);
	norflaship_mem_read_bus_unlock(id, boot_nor_mrb_status[id]);
	return 0;
}

SRAM(norflash_get_id)
int32_t norflash_get_id(uint32_t id, uint8_t *data, uint32_t length)
{
	(void)norflash_pre_operation_suspend(id, 0U);
	uint32_t count = MIN(length, 3U);

	norflaship_clear_fifos(id);
	norflaship_ext_rx_cmd(id, 0x9fU, 0U, count);
	norflaship_rxfifo_count_wait(id, count);
	for (uint32_t i = 0U; i < count; i++) {
		data[i] = norflaship_read_rxfifo(id);
	}
	(void)norflash_post_operation_suspend(id, 0U, 0U);
	return 0;
}

SRAM(norflash_init_sample_delay_by_div)
int32_t norflash_init_sample_delay_by_div(uint32_t id, uint32_t divider)
{
	if (divider == 0U) {
		return -1;
	}
	uint32_t delay = divider == 1U   ? 1U
			 : divider == 2U ? (boot_nor_preferred_idx_adj[id] != 0U ? 4U : 3U)
					 : 4U;

	norflaship_samdly(id, delay);
	return 0;
}
