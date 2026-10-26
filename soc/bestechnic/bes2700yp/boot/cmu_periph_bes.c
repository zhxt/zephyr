/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/arch/common/sys_io.h>

#include "cmu_periph_native.h"

#ifdef CMU_PERIPH_MMIO_MODEL
extern volatile uint32_t cmu_periph_model_uart;
#define CMU_UART_ADDRESS ((uintptr_t)&cmu_periph_model_uart)
#else
#define CMU_UART_ADDRESS ((uintptr_t)0x40000070U)
#endif

__attribute__((section(".boot_text_flash.boot_cmu_uart_modify"), noinline))
void boot_cmu_uart_modify(uint32_t clear_mask, uint32_t set_mask)
{
	uint32_t value = sys_read32(CMU_UART_ADDRESS);

	sys_write32((value & ~clear_mask) | set_mask, CMU_UART_ADDRESS);
}
