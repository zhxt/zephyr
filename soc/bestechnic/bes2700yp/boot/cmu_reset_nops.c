/* SPDX-License-Identifier: Apache-2.0 */

#include "cmu_reset_native.h"

#ifdef CMU_RESET_NOP_HOST
#include "tests/cmu_reset_nop_host.h"
#define RESET_NOP() cmu_reset_model_nop()
#else
#include "arch.h"
#define RESET_NOP() __NOP()
#endif

__attribute__((section(".boot_text_flash.boot_cmu_reset_nops"), noinline))
void boot_cmu_reset_nops(uint32_t count)
{
	if (count == 2U) {
		RESET_NOP();
		RESET_NOP();
	} else if (count == 4U) {
		RESET_NOP();
		RESET_NOP();
		RESET_NOP();
		RESET_NOP();
	}
}
