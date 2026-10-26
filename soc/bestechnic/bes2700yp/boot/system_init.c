/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "arch.h"
#include "runtime.h"

/* Preserve the loader CPU policy using architectural CMSIS definitions only. */
__attribute__((section(".boot_text_flash.cpu_init"))) void boot_cpu_init(void)
{
	SCB->CPACR |= (3UL << 20U) | (3UL << 22U);
	__DSB();
	__ISB();
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
	SCB->CCR |= SCB_CCR_STKOFHFNMIGN_Msk;
	SCB->CCR &= ~SCB_CCR_UNALIGN_TRP_Msk;
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;
	NVIC_SetPriority(UsageFault_IRQn, 0U);
	SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
	NVIC_SetPriority(BusFault_IRQn, 0U);
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
	NVIC_SetPriority(MemoryManagement_IRQn, 0U);
	NVIC_SetPriority(DebugMonitor_IRQn, 0U);
}
