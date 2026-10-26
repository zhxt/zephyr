/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "flash_readonly.h"
#include "flaship_native.h"
#include "nor_native.h"

#ifdef BOOT_FLASH_READONLY_HOST
void bth_puts(const char *text);
void bth_field(const char *name, uint32_t value);
void bth_end(void);
#else
#include "boot_contract.h"
#endif

#define SRAM(name) __attribute__((section(".boot_text_sram." #name), noinline))
#define EXPECTED_JEDEC 0x00c86017U
#define DEVICE_BYTES   0x00800000U
#define READ_DIVIDER   8U
#define SAMPLE_DELAY   4U

struct readonly_state {
	uint32_t calls;
	uint32_t controller;
	uint32_t id_reads;
	uint32_t jedec;
	uint32_t rc;
	uint32_t sr1;
	uint32_t sr2;
	uint32_t sr1_after;
	uint32_t sr2_after;
	uint32_t divider;
	uint32_t actual_crc;
	uint32_t expected_crc;
	uint32_t bytes;
	uint32_t verified;
};

__attribute__((section(".boot_bss.flash_readonly")))
static volatile struct readonly_state state;

/*
 * The public GD25LQ64C command set supplies 9F/05/35 and standard 03 reads.
 * Controller access and reset retain the separately qualified BES backend.
 * No chip-specific quad, protection, status-write or high-speed policy is used.
 */
SRAM(boot_flash_readonly_prepare)
int boot_flash_readonly_prepare(uint32_t controller)
{
	uint8_t id[3] = {0U, 0U, 0U};

	state.calls++;
	state.controller = controller;
	state.rc = 1U;
	state.verified = 0U;
	if (controller != 0U || state.calls != 1U) {
		return 1;
	}
	norflaship_busy_wait(controller);
	norflaship_set_idle_io_dir(controller, 0U);
	norflaship_nand_sel(controller, 0U);
	norflash_reset(controller);
	/* Keep the previously exercised low-rate controller timing. */
	boot_nor_div_read[controller] = READ_DIVIDER;
	boot_nor_div_others[controller] = READ_DIVIDER;
	norflaship_div(controller, READ_DIVIDER);
	norflaship_samdly(controller, SAMPLE_DELAY);
	norflaship_rdcmd(controller, 0x03U);
	state.id_reads++;
	if (norflash_get_id(controller, id, sizeof(id)) != 0) {
		state.rc = 2U;
		return 2;
	}
	state.jedec = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
	if (state.jedec != EXPECTED_JEDEC) {
		state.rc = 3U;
		return 3;
	}
	state.sr1 = norflash_read_status_s0_s7(controller);
	state.sr2 = norflash_read_status_s8_s15(controller);
	norflaship_cmd_done(controller);
	state.divider = norflaship_get_div(controller);
	if ((state.sr1 & 1U) != 0U || state.divider != READ_DIVIDER) {
		state.rc = 4U;
		return 4;
	}
	state.rc = 0U;
	return 0;
}

SRAM(boot_flash_readonly_finish)
int boot_flash_readonly_finish(uint32_t actual_crc, uint32_t expected_crc, uint32_t bytes)
{
	state.verified = 0U;
	if (state.calls != 1U || state.rc != 0U) {
		if (state.rc == 0U) {
			state.rc = 5U;
		}
		return (int)state.rc;
	}
	state.actual_crc = actual_crc;
	state.expected_crc = expected_crc;
	state.bytes = bytes;
	state.sr1_after = norflash_read_status_s0_s7(0U);
	state.sr2_after = norflash_read_status_s8_s15(0U);
	norflaship_cmd_done(0U);
	state.divider = norflaship_get_div(0U);
	if (actual_crc != expected_crc || bytes == 0U || bytes > 0x30000U) {
		state.rc = 6U;
	} else if (state.sr1 != state.sr1_after || state.sr2 != state.sr2_after ||
		   state.divider != READ_DIVIDER) {
		state.rc = 7U;
	} else {
		state.verified = 1U;
	}
	return (int)state.rc;
}

void boot_flash_readonly_dump(void)
{
	bth_puts("zephyr_bth flash_readonly version=1 provider=spi-readonly");
	bth_field(" calls=", state.calls);
	bth_field(" controller=", state.controller);
	bth_field(" id_reads=", state.id_reads);
	bth_field(" jedec=", state.jedec);
	bth_field(" rc=", state.rc);
	/* This is one JEDEC device, not the aggregate package capacity. */
	bth_field(" device_bytes=", DEVICE_BYTES);
	bth_field(" read_opcode=", 3U);
	bth_field(" divider=", state.divider);
	bth_field(" sample=", SAMPLE_DELAY);
	bth_field(" sr1=", state.sr1);
	bth_field(" sr2=", state.sr2);
	bth_field(" sr1_after=", state.sr1_after);
	bth_field(" sr2_after=", state.sr2_after);
	bth_field(" crc=", state.actual_crc);
	bth_field(" expected_crc=", state.expected_crc);
	bth_field(" bytes=", state.bytes);
	bth_field(" verified=", state.verified);
	bth_puts(" full_hal_init=0 !\r\n");
}
