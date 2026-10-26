/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>

#include "cmu_timer_native.h"

#ifdef CMU_TIMER_MMIO_MODEL
extern volatile uint32_t cmu_timer_model_register;
#define CMU_TIMER_ADDRESS ((uintptr_t)&cmu_timer_model_register)
#else
#define CMU_TIMER_ADDRESS ((uintptr_t)0x4000005cU)
#endif

__attribute__((section(".boot_text_flash.boot_cmu_timer_modify"), noinline))
void boot_cmu_timer_modify(uint32_t clear_mask, uint32_t set_mask)
{
	uint32_t value = sys_read32(CMU_TIMER_ADDRESS);

	sys_write32((value & ~clear_mask) | set_mask, CMU_TIMER_ADDRESS);
}
