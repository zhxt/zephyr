/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES2700YP_BOOT_FLASHIP_NATIVE_H_
#define BES2700YP_BOOT_FLASHIP_NATIVE_H_

#include <stdint.h>

/* Fixed loader ABI: controller IDs zero and one; this is not a Flash API. */
void norflaship_continuous_read_off(uint32_t id);
void norflaship_write_txfifo(uint32_t id, const uint8_t *buffer, uint32_t length);
uint8_t norflaship_read_rxfifo(uint32_t id);
void norflaship_cmd_addr(uint32_t id, uint32_t command, uint32_t address);
void norflaship_ext_tx_cmd(uint32_t id, uint32_t command, uint32_t length);
void norflaship_ext_rx_cmd(uint32_t id, uint32_t command, uint32_t tx_length, uint32_t rx_length);
void norflaship_cmd_done(uint32_t id);
void norflaship_rxfifo_count_wait(uint32_t id, uint32_t count);
void norflaship_busy_wait(uint32_t id);
uint32_t norflaship_is_busy(uint32_t id);
void norflaship_clear_fifos(uint32_t id);
void norflaship_clear_rxfifo(uint32_t id);
void norflaship_div(uint32_t id, uint32_t value);
uint32_t norflaship_get_div(uint32_t id);
void norflaship_samdly(uint32_t id, uint32_t value);
void norflaship_dual_mode(uint32_t id, uint32_t value);
void norflaship_quad_mode(uint32_t id, uint32_t value);
void norflaship_4byteaddr_mode(uint32_t id, uint32_t value);
void norflaship_nand_sel(uint32_t id, uint32_t value);
void norflaship_rdcmd(uint32_t id, uint32_t value);
void norflaship_qrdcmd(uint32_t id, uint32_t value);
void norflaship_set_idle_io_dir(uint32_t id, uint32_t value);
void norflaship_dtr_mode(uint32_t id, uint32_t value);
uint32_t norflaship_mem_read_bus_lock(uint32_t id);
void norflaship_mem_read_bus_unlock(uint32_t id, uint32_t key);

void boot_flaship_write8(uintptr_t address, uint8_t value);

#endif /* BES2700YP_BOOT_FLASHIP_NATIVE_H_ */
