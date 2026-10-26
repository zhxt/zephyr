/* SPDX-License-Identifier: Apache-2.0 */

#include "sysfreq_native.h"

extern uint32_t irq_masked_addr[2];

#ifdef SYSFREQ_IRQ_HOST
#include "tests/sysfreq_irq_host.h"
#else
#include "arch.h"

static inline uint32_t sysfreq_diagnostic_site(void)
{
	uint32_t site;

	__asm__ volatile("mov %0, pc" : "=r" (site));
	return site;
}

static inline void sysfreq_diagnostic_record(uint32_t site, uint32_t caller)
{
	/* Keep the two-word record in one instruction before changing BASEPRI. */
	__asm__ volatile("strd %1, %2, [%0]" : : "r" (irq_masked_addr),
			 "r" (site), "r" (caller) : "memory");
}
#endif

__attribute__((section(".boot_text_flash.boot_sysfreq_irq_lock"), noinline))
uint32_t boot_sysfreq_irq_lock(uint32_t caller)
{
	uint32_t key = __get_BASEPRI();

	if (key != 0x40U) {
		sysfreq_diagnostic_record(sysfreq_diagnostic_site(), caller);
	}
	/* Preserve the selected loader's unconditional threshold, including nesting. */
	__set_BASEPRI(0x40U);
	return key;
}

__attribute__((section(".boot_text_flash.boot_sysfreq_irq_unlock"), noinline))
void boot_sysfreq_irq_unlock(uint32_t key)
{
	volatile uint32_t *diagnostic = irq_masked_addr;

	if (key == 0U) {
		diagnostic[0] = UINT32_MAX;
	}
	__set_BASEPRI(key);
}
