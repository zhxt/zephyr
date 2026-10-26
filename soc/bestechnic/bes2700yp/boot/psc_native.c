/* SPDX-License-Identifier: Apache-2.0 */

#include "psc_native.h"

#define EARLY_STATE(name) __attribute__((section(".boot_bss.psc_" #name)))
#define EARLY_CODE(name) __attribute__((section(".boot_text_flash." #name), noinline, noclone))
#define SRAM_CODE(name) __attribute__((section(".boot_text_sram." #name), noinline, noclone))

EARLY_STATE(bth_pu) volatile uint16_t boot_psc_bth_shr_pu;
EARLY_STATE(bth_pd) volatile uint16_t boot_psc_bth_shr_pd;
EARLY_STATE(m55_tcm) volatile uint16_t boot_psc_m55_tcm;
EARLY_STATE(m55_shared) volatile uint16_t boot_psc_m55_shared;
EARLY_STATE(dsp_tcm) volatile uint16_t boot_psc_dsp_tcm;
EARLY_STATE(dsp_shared) volatile uint16_t boot_psc_dsp_shared;
EARLY_STATE(sys_map) volatile uint8_t boot_psc_sys_map;

/* Unlike the seven fields above, these two bytes belong to normal CRT BSS. */
volatile uint8_t boot_psc_display_map;
volatile uint8_t boot_psc_storage_map;

__attribute__((always_inline))
static inline void update(uint32_t address, uint32_t clear, uint32_t set)
{
	uint32_t value = boot_psc_read32(address);

	boot_psc_write32(address, (value & ~clear) | set);
}

SRAM_CODE(boot_psc_ram_config)
static void ram_config(uint32_t address, uint32_t selected, uint32_t mode, uint32_t sleep)
{
	uint32_t clear;
	uint32_t set;

	if (selected == 0U) {
		return;
	}
	if (mode != 0U) {
		clear = selected | (selected << 10U);
		set = selected << 20U;
	} else if (sleep != 0U) {
		clear = selected | (selected << 20U);
		set = selected << 10U;
	} else {
		clear = selected | (selected << 10U) | (selected << 20U);
		set = 0U;
	}
	update(address, clear, set);
}

SRAM_CODE(hal_cmu_sys_ram_active_cfg)
void hal_cmu_sys_ram_active_cfg(uint32_t map, uint32_t mode)
{
	ram_config(0x4008008cU, map & 0x3ffU, mode, 0U);
	ram_config(0x40080094U, (map >> 10U) & 0x3fU, mode, 0U);
}

SRAM_CODE(hal_cmu_sys_ram_sleep_cfg)
void hal_cmu_sys_ram_sleep_cfg(uint32_t map, uint32_t mode)
{
	ram_config(0x40080088U, map & 0x3ffU, mode, 1U);
	ram_config(0x40080090U, (map >> 10U) & 0x3fU, mode, 1U);
}

EARLY_CODE(boot_psc_shutdown)
static void shutdown(uint32_t address)
{
	/* Every write uses this one snapshot, rather than a new register read. */
	uint32_t value = boot_psc_read32(address);

	boot_psc_write32(address, value | 0xcafe02f1U);
	boot_psc_write32(address, value | 0xcafe02f3U);
	boot_psc_write32(address, value | 0xcafe02f7U);
	boot_psc_write32(address, value | 0xcafe02ffU);
	boot_psc_write32(address, value | 0xcafe03ffU);
}

EARLY_CODE(boot_psc_shutdown_small)
static void shutdown_small(uint32_t address)
{
	uint32_t value = boot_psc_read32(address);

	boot_psc_write32(address, value | 0xcafe00f1U);
	boot_psc_write32(address, value | 0xcafe00f3U);
	boot_psc_write32(address, value | 0xcafe00f7U);
	boot_psc_write32(address, value | 0xcafe00ffU);
}

EARLY_CODE(hal_psc_sys_disable)
void hal_psc_sys_disable(uint32_t user)
{
	if (user > 3U) {
		return;
	}
	if (user != 3U) {
		uint8_t before = boot_psc_sys_map;
		uint8_t after;

		if ((before & (1U << user)) == 0U) {
			return;
		}
		after = before & ~(1U << user);
		boot_psc_sys_map = after;
		if ((after & 3U) != 0U) {
			return;
		}
		if (after == 4U) {
			update(0x40085018U, 0U, 0xcafe0300U);
			return;
		}
	}
	shutdown(0x40085018U);
}

EARLY_CODE(hal_psc_sys_m55_disable)
void hal_psc_sys_m55_disable(void)
{
	uint16_t map;
	uint32_t value;
	uint32_t shared;

	shutdown(0x40085094U);
	map = boot_psc_m55_tcm;
	hal_cmu_sys_ram_active_cfg(map, 1U);
	hal_cmu_sys_ram_sleep_cfg(map, 1U);
	value = boot_psc_read32(0x40085168U);
	shared = boot_psc_m55_shared & 0x7ffU;
	boot_psc_write32(0x40085168U, value | shared | (shared << 11U) | 0xca000000U);
	hal_psc_sys_disable(0U);
}

EARLY_CODE(hal_psc_sys_dsp_disable)
void hal_psc_sys_dsp_disable(void)
{
	uint16_t map;
	uint32_t value;
	uint32_t shared;

	shutdown(0x400850a8U);
	map = boot_psc_dsp_tcm;
	hal_cmu_sys_ram_active_cfg(map, 1U);
	hal_cmu_sys_ram_sleep_cfg(map, 1U);
	value = boot_psc_read32(0x40085168U);
	shared = boot_psc_dsp_shared & 0x7ffU;
	boot_psc_write32(0x40085168U, value | shared | (shared << 11U) | 0xca000000U);
	hal_psc_sys_disable(1U);
}

EARLY_CODE(hal_psc_sens_disable)
void hal_psc_sens_disable(void)
{
	shutdown_small(0x40085080U);
	shutdown(0x40085058U);
}

__attribute__((always_inline))
static inline void shared_disable(uint32_t user, uint32_t force, volatile uint8_t *map,
				 uint32_t address, uint32_t caller)
{
	uint32_t key;

	if (user > force) {
		return;
	}
	key = boot_psc_irq_lock(caller);
	if (user != force) {
		uint8_t after = *map & ~(1U << user);

		*map = after;
		if (after != 0U) {
			goto restore;
		}
	}
	shutdown(address);
restore:
	boot_psc_irq_unlock(key);
}

EARLY_CODE(hal_psc_display_disable)
void hal_psc_display_disable(uint32_t user)
{
	shared_disable(user, 5U, &boot_psc_display_map, 0x400850bcU,
		       (uint32_t)(uintptr_t)__builtin_return_address(0));
}

EARLY_CODE(hal_psc_storage_disable)
void hal_psc_storage_disable(uint32_t user)
{
	shared_disable(user, 4U, &boot_psc_storage_map, 0x400850e4U,
		       (uint32_t)(uintptr_t)__builtin_return_address(0));
}

EARLY_CODE(hal_psc_init)
void hal_psc_init(void)
{
	uint16_t m55;
	uint16_t remaining;

	boot_psc_write32(0x400850fcU, 0x03000000U);
	boot_psc_write32(0x40085100U, 0x400cU);
	boot_psc_write32(0x40085114U, 0U);
	boot_psc_write32(0x40085118U, 0x800U);
	boot_psc_write32(0x4008512cU, 0U);
	boot_psc_write32(0x40085130U, 0x1010U);
	boot_psc_write32(0x40085144U, 0U);
	boot_psc_write32(0x40085148U, 0x2102U);
	boot_psc_dsp_shared = 0U;
	m55 = boot_psc_m55_tcm | 0x3f8U;
	boot_psc_m55_shared = 0U;
	boot_psc_m55_tcm = m55;
	boot_psc_write32(0x40085168U, 0xca0007ffU);
	boot_psc_write32(0x40085174U, 0x1fU);
	remaining = (uint16_t)~(m55 | boot_psc_dsp_tcm);
	hal_cmu_sys_ram_active_cfg(remaining, 1U);
	hal_cmu_sys_ram_sleep_cfg(remaining, 1U);
	update(0x40085168U, 0U, 0xca07f8ffU);
	update(0x40085130U, 0U, 0x86e1U);
	boot_psc_sys_map = 7U;
	boot_psc_write32(0x40085018U, 0xcafe02f0U);
	hal_psc_sys_m55_disable();
	hal_psc_sys_dsp_disable();
	hal_psc_sens_disable();
	shutdown_small(0x400850d0U);
	hal_psc_display_disable(5U);
	hal_psc_storage_disable(4U);
	update(0x40085060U, 0U, 0x18U);
	boot_psc_write32(0x40085008U, 0U);
	boot_psc_write32(0x4008500cU, 0U);
	update(0x40085180U, 0U, 0xf0U);
}

EARLY_CODE(hal_psc_core_init_ram_ret_mask)
void hal_psc_core_init_ram_ret_mask(void)
{
	boot_psc_bth_shr_pu = 0U;
	boot_psc_bth_shr_pd = 0U;
}

__attribute__((section(".text.hal_psc_ram_ret_during_reset"), noinline))
void hal_psc_ram_ret_during_reset(void)
{
	boot_psc_write32(0x40080088U, 0x000ffc00U);
	boot_psc_write32(0x40080090U, 0x000ffc00U);
	boot_psc_write32(0x400800b0U, 0x000ffc00U);
	boot_psc_write32(0x400800e8U, 0x000ffc00U);
	boot_psc_write32(0x40080024U, 0x000ffc00U);
	boot_psc_write32(0x4008002cU, 0x000ffc00U);
	boot_psc_write32(0x4008016cU, 0x000ffc00U);
	(void)boot_psc_read32(0x4008016cU);
	boot_psc_sync();
	boot_psc_write32(0x40085168U, 0xca000000U);
	boot_psc_write32(0x40085174U, 0U);
	(void)boot_psc_read32(0x40085174U);
	boot_psc_sync();
}
