/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>
#include <zephyr/arch/common/sys_bitops.h>

#include "cmu_setup_native.h"

/* Fixed-build diagnostic bus operation; no public BES bit meaning is claimed. */
#ifdef CMU_SETUP_MMIO_MODEL
extern volatile uint32_t cmu_setup_model_fault_register;
#define CMU_SETUP_FAULT_ADDRESS ((uintptr_t)&cmu_setup_model_fault_register)
#else
#define CMU_SETUP_FAULT_ADDRESS ((uintptr_t)0x500000f4U)
#endif

__attribute__((noreturn, section(".boot_text_flash.boot_clock_fault_halt")))
void boot_clock_fault_halt(void)
{
	sys_set_bits(CMU_SETUP_FAULT_ADDRESS, 0x80000000U);
	for (;;) {
		volatile uint32_t spin = 0U;

		spin++;
	}
}
