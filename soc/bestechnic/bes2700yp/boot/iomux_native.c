/* SPDX-License-Identifier: Apache-2.0 */

#include "iomux_native.h"

/* Early CRT clears this byte; normal CRT must preserve an existing owner. */
__attribute__((section(".boot_bss.iomux_memsc")))
volatile uint8_t boot_iomux_got_memsc;

__attribute__((always_inline))
static inline void update(uint32_t address, uint32_t clear, uint32_t set)
{
	uint32_t value = boot_iomux_read32(address);

	boot_iomux_write32(address, (value & ~clear) | set);
}

__attribute__((section(".boot_text_sram.boot_iomux_lock"), noinline))
uint32_t boot_iomux_lock(void)
{
	uint32_t key = boot_iomux_irq_lock((uint32_t)(uintptr_t)__builtin_return_address(0));

	if (boot_iomux_got_memsc == 0U) {
		while (hal_memsc_aon_lock(0U) == 0) {
		}
		boot_iomux_got_memsc = 1U;
		key |= 0x80000000U;
	}
	return key;
}

/* This helper is used only for the selected pins and fixed drive value one. */
__attribute__((section(".boot_text_flash.boot_iomux_drive"), noinline))
static void drive(uint32_t pin)
{
	uint32_t address;
	uint32_t shift;

	if (pin < 40U) {
		address = 0x50060044U;
		shift = (pin - 32U) * 2U;
	} else if (pin < 44U) {
		address = 0x50060048U;
		shift = (pin - 40U) * 2U;
	} else if (pin < 72U) {
		address = 0x5006004cU;
		shift = (pin - 56U) * 2U;
	} else {
		address = 0x5006005cU;
		shift = (pin - 72U) * 2U;
	}
	update(address, 3U << shift, 1U << shift);
}

__attribute__((section(".boot_text_flash.boot_iomux_default_pin"), noinline))
static void default_pin(uint32_t pin)
{
	uint32_t function;
	uint32_t clear_address;
	uint32_t enable_address;
	uint32_t clear_bit;
	uint32_t enable_bit;
	uint32_t shift;

	if (pin < 24U) {
		function = 0x40086004U + (pin / 8U) * 4U;
		shift = (pin & 7U) * 4U;
		clear_address = 0x4008602cU;
		enable_address = 0x40086030U;
		clear_bit = pin;
		enable_bit = pin;
	} else if (pin < 32U) {
		function = 0x50060058U;
		shift = (pin - 24U) * 4U;
		clear_address = 0x50060060U;
		enable_address = clear_address;
		clear_bit = pin - 24U;
		enable_bit = pin - 16U;
	} else if (pin < 64U) {
		function = 0x50060004U + ((pin - 32U) / 8U) * 4U;
		shift = (pin & 7U) * 4U;
		clear_address = 0x50060024U;
		enable_address = 0x50060028U;
		clear_bit = pin - 32U;
		enable_bit = clear_bit;
	} else {
		function = pin < 72U ? 0x50060014U : 0x50060038U;
		shift = (pin & 7U) * 4U;
		clear_address = 0x5006002cU;
		enable_address = clear_address;
		clear_bit = pin < 72U ? pin - 64U : pin - 56U;
		enable_bit = clear_bit + 8U;
	}
	if (((boot_iomux_read32(function) >> shift) & 15U) != 15U) {
		return;
	}
	/* Preserve both fresh reads, including when the two addresses are equal. */
	update(clear_address, 1U << clear_bit, 0U);
	update(enable_address, 0U, 1U << enable_bit);
	if (pin >= 24U) {
		uint32_t pair = (pin - 24U) / 8U;
		uint32_t route = pair < 2U ? 0x40080034U : pair < 4U ?
			0x40080058U : pair < 6U ? 0x40080070U : 0x40080144U;
		uint32_t bit = (pin & 7U) + (pair & 1U) * 16U;

		update(route, 1U << bit, 1U << (bit + 8U));
	}
}

__attribute__((section(".boot_text_flash.hal_iomux_set_default_config"), noinline))
void hal_iomux_set_default_config(void)
{
	boot_iomux_write32(0x40086008U, UINT32_MAX);
	boot_iomux_write32(0x50060038U, UINT32_MAX);
	boot_iomux_write32(0x40086070U, 0U);
	boot_iomux_write32(0x50060064U, 0U);
	for (uint32_t pin = 0U; pin < 80U; pin++) {
		default_pin(pin);
	}
	if (hal_cmu_get_aon_revision_id() > 2U) {
		for (uint32_t pin = 32U; pin < 44U; pin++) {
			drive(pin);
		}
		for (uint32_t pin = 64U; pin < 80U; pin++) {
			drive(pin);
		}
	}
	boot_iomux_write32(0x40086044U, 0x0002aaaaU);
	boot_iomux_sync();
}

__attribute__((section(".text.hal_iomux_set_uart0"), noinline))
void hal_iomux_set_uart0(void)
{
	uint32_t key = boot_iomux_lock();

	update(0x40086048U, 0U, 3U);
	update(0x4008600cU, 0xff00U, 0x4400U);
	update(0x4008602cU, 0U, 0x40000U);
	update(0x4008602cU, 0x80000U, 0U);
	update(0x40086030U, 0xc0000U, 0U);
	if ((key & 0x80000000U) != 0U) {
		boot_iomux_got_memsc = 0U;
		hal_memsc_aon_unlock(0U);
		key &= ~0x80000000U;
	}
	boot_iomux_irq_unlock(key);
}
