/* SPDX-License-Identifier: Apache-2.0 */

#include "iomux_native.h"

extern uint32_t irq_masked_addr[2];

#ifdef IOMUX_IRQ_HOST
#include "tests/iomux_irq_host.h"
#else
#include "arch.h"

static inline uint32_t iomux_diagnostic_site(void)
{
	uint32_t site;

	__asm__ volatile("mov %0, pc" : "=r" (site));
	return site;
}

static inline void iomux_diagnostic_record(uint32_t site, uint32_t caller)
{
	__asm__ volatile("strd %1, %2, [%0]" : : "r" (irq_masked_addr),
			 "r" (site), "r" (caller) : "memory");
}
#endif

__attribute__((section(".boot_text_sram.boot_iomux_irq_lock"), noinline))
uint32_t boot_iomux_irq_lock(uint32_t caller)
{
	uint32_t key = __get_BASEPRI();

	if (key != 0x40U) {
		iomux_diagnostic_record(iomux_diagnostic_site(), caller);
	}
	__set_BASEPRI(0x40U);
	return key;
}

__attribute__((section(".text.boot_iomux_irq_unlock"), noinline))
void boot_iomux_irq_unlock(uint32_t key)
{
	volatile uint32_t *diagnostic = irq_masked_addr;

	if (key == 0U) {
		diagnostic[0] = UINT32_MAX;
	}
	__set_BASEPRI(key);
}

__attribute__((section(".boot_text_flash.boot_iomux_sync"), noinline))
void boot_iomux_sync(void)
{
	__DSB();
}
