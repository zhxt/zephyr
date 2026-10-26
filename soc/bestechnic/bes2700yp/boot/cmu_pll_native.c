/* SPDX-License-Identifier: Apache-2.0 */

#include "cmu_pll_native.h"
#include "cmu_gate_native.h"
#include "cmu_timer_native.h"
#include "timer_native.h"

volatile uint16_t boot_cmu_pll_users[3]
	__attribute__((section(".boot_bss.boot_cmu_pll_users")));

struct pll_location {
	uintptr_t enable;
	uint32_t power;
	uint32_t ready;
	uint32_t output;
};

__attribute__((section(".boot_text_flash.boot_cmu_pll_location"), noinline))
static void pll_location(uint32_t pll, struct pll_location *location)
{
	location->enable = pll == 0U ? 0x40000060U : 0x40080004U;
	location->power = pll == 0U ? 0x40000U : pll == 1U ? 0x8000U : 0x800U;
	location->ready = pll == 0U ? 0x20000U : pll == 1U ? 0U : 0x400U;
	location->output = pll == 0U ? 0x800U : pll == 1U ? 0U : 2U;
}

__attribute__((section(".boot_text_flash.boot_cmu_pll_enable"), noinline, noclone))
static int pll_enable(uint32_t pll, uint32_t user, uint32_t caller, uint32_t internal)
{
	struct pll_location location;
	uint32_t key;
	uint32_t start;
	uint16_t users;

	if (internal != 0U) {
		caller = (uint32_t)(uintptr_t)__builtin_return_address(0);
	}
	pll_location(pll, &location);
	key = boot_cmu_timer_irq_lock(caller);
	users = boot_cmu_pll_users[pll];
	if (users == 0U || user == 11U) {
		boot_cmu_gate_write(location.enable, location.power);
		hal_sys_timer_delay_us(20U);
		/* The timer callback can publish state; reload rather than use the snapshot. */
		if (user != 11U) {
			boot_cmu_pll_users[pll] = boot_cmu_pll_users[pll] | (1U << user);
		}
	} else {
		boot_cmu_pll_users[pll] = users | (1U << user);
	}
	boot_cmu_timer_irq_unlock(key);
	start = hal_sys_timer_get();
	for (;;) {
		if ((boot_cmu_gate_read(location.enable) & location.ready) != 0U) {
			break;
		}
		if ((uint32_t)(hal_sys_timer_get() - start) > 4U) {
			break;
		}
	}
	/* Preserve the zero-valued PLL1 commands and success after a timeout. */
	boot_cmu_gate_write(location.enable, location.ready);
	boot_cmu_gate_write(0x400800a0U, location.output);
	return 0;
}

__attribute__((section(".boot_text_flash.hal_cmu_pll_enable"), noinline))
int hal_cmu_pll_enable(uint32_t pll, uint32_t user)
{
	if (pll > 2U) {
		return 1;
	}
	if (user > 11U) {
		return 2;
	}
	return pll_enable(pll, user, (uint32_t)(uintptr_t)__builtin_return_address(0), 0U);
}

__attribute__((section(".boot_text_flash.hal_cmu_pll_disable"), noinline))
int hal_cmu_pll_disable(uint32_t pll, uint32_t user)
{
	struct pll_location location;
	uint32_t key;
	uint16_t users = 0U;

	if (pll > 2U) {
		return 1;
	}
	if (user > 11U) {
		return 2;
	}
	pll_location(pll, &location);
	key = boot_cmu_timer_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));
	if (user != 11U) {
		users = boot_cmu_pll_users[pll] & ~(1U << user);
		boot_cmu_pll_users[pll] = users;
	}
	if (user == 11U || users == 0U) {
		boot_cmu_gate_write(0x400800a4U, location.output);
		boot_cmu_gate_write(location.enable + 4U, location.ready);
		boot_cmu_gate_write(location.enable + 4U, location.power);
	}
	boot_cmu_timer_irq_unlock(key);
	return 0;
}

__attribute__((section(".boot_text_flash.hal_cmu_init_pll_selection"), noinline))
void hal_cmu_init_pll_selection(void)
{
	(void)pll_enable(0U, 0U, 0U, 1U);
	hal_cmu_init_axi_pll_selection();
}
