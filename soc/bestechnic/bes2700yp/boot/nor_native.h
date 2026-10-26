/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES2700YP_BOOT_NOR_NATIVE_H_
#define BES2700YP_BOOT_NOR_NATIVE_H_

#include <stddef.h>
#include <stdint.h>

/* Early state for the fixed standard-SPI read path. */
extern uint8_t boot_nor_div_read[2];
extern uint8_t boot_nor_div_others[2];
extern uint8_t boot_nor_preferred_idx_adj[2];
extern uint32_t boot_nor_mrb_status[2];

void boot_nor_set_read_div(uint32_t id);
uint8_t norflash_read_status_s0_s7(uint32_t id);
uint8_t norflash_read_status_s8_s15(uint32_t id);
void norflash_reset(uint32_t id);
int32_t norflash_pre_operation_suspend(uint32_t id, uint32_t suspend);
int32_t norflash_post_operation_suspend(uint32_t id, uint32_t suspend, uint32_t resume);
int32_t norflash_get_id(uint32_t id, uint8_t *data, uint32_t length);
int32_t norflash_init_sample_delay_by_div(uint32_t id, uint32_t divider);

#endif /* BES2700YP_BOOT_NOR_NATIVE_H_ */
