/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>

#include "main_cmu_frequency_native.h"

extern uint32_t irq_masked_addr[2];

#ifdef MAIN_CMU_FREQUENCY_HOST
#include "tests/cmu_timer_irq_host.h"
void __DSB(void);
#else
#include "arch.h"

static inline uint32_t cmu_timer_diagnostic_site(void)
{
	uint32_t site;

	__asm__ volatile("mov %0, pc" : "=r" (site));
	return site;
}

static inline void cmu_timer_diagnostic_record(uint32_t site, uint32_t caller)
{
	__asm__ volatile("strd %1, %2, [%0]" : : "r" (irq_masked_addr),
			 "r" (site), "r" (caller) : "memory");
}
#endif

#ifdef MAIN_CMU_FREQUENCY_MMIO_MODEL
uintptr_t main_cmu_frequency_model_address(uintptr_t address);
#define FREQUENCY_ADDRESS(address) main_cmu_frequency_model_address(address)
#else
#define FREQUENCY_ADDRESS(address) (address)
#endif

#define SRAM_CODE(name) __attribute__((section(".boot_text_sram." #name), noinline))

/* The complete backend stays in SRAM while the BTH Flash clock is changing. */
SRAM_CODE(boot_main_cmu_frequency_read)
uint32_t boot_main_cmu_frequency_read(uintptr_t address)
{
	return sys_read32(FREQUENCY_ADDRESS(address));
}

SRAM_CODE(boot_main_cmu_frequency_write)
void boot_main_cmu_frequency_write(uintptr_t address, uint32_t value)
{
	sys_write32(value, FREQUENCY_ADDRESS(address));
}

SRAM_CODE(boot_main_cmu_frequency_irq_lock)
uint32_t boot_main_cmu_frequency_irq_lock(uint32_t caller)
{
	uint32_t key = __get_BASEPRI();

	if (key != 0x40U) {
		cmu_timer_diagnostic_record(cmu_timer_diagnostic_site(), caller);
	}
	__set_BASEPRI(0x40U);
	return key;
}

SRAM_CODE(boot_main_cmu_frequency_irq_unlock)
void boot_main_cmu_frequency_irq_unlock(uint32_t key)
{
	volatile uint32_t *diagnostic = irq_masked_addr;

	if (key == 0U) {
		diagnostic[0] = UINT32_MAX;
	}
	__set_BASEPRI(key);
}

SRAM_CODE(boot_main_cmu_frequency_barrier)
void boot_main_cmu_frequency_barrier(void)
{
	__DSB();
}
