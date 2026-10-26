/* SPDX-License-Identifier: Apache-2.0 */

#include "timer_native.h"

#ifdef BOOT_TIMER_NATIVE_HOST
#include "tests/timer_native_host.h"
#else
#include "arch.h"
#define TIMER_READ32(address) (*(__IM uint32_t *)(uintptr_t)(address))
#define TIMER_WRITE32(address, value) \
	(*(__IO uint32_t *)(uintptr_t)(address) = (value))
#endif

#ifndef __section
#define __section(name) __attribute__((__section__(name)))
#endif

/* Local BES experiment mappings; no public BES integration specification.
 * Fast VALUE/CTRL offsets and enable/width bits follow the public CMSDK
 * protocol described by Zephyr drivers/counter/dualtimer_cmsdk_apb.h.
 */
#define SLOW_COUNTER 0x40080050U
#define FAST_VALUE 0x40002004U
#define FAST_CONTROL 0x40002008U
#define TIMER_ENABLE (1U << 7)
#define TIMER_SIZE_32 (1U << 1)

__section(".boot_text_sram.boot_timer_slow_read32")
uint32_t boot_timer_slow_read32(void)
{
	return TIMER_READ32(SLOW_COUNTER);
}

__section(".boot_text_sram.boot_timer_fast_read32")
uint32_t boot_timer_fast_read32(void)
{
	return TIMER_READ32(FAST_VALUE);
}

__section(".boot_text_flash.boot_timer_hw_open")
void boot_timer_hw_open(void)
{
	(void)hal_cmu_fast_timer_init();
	hal_cmu_timer1_select_fast();
	TIMER_WRITE32(FAST_CONTROL, TIMER_ENABLE | TIMER_SIZE_32);
	hal_cmu_timer2_select_fast();
}
