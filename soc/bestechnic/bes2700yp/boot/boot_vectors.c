/*
 * Copyright (c) 2009-2021 Arm Limited. All rights reserved.
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Weak exception aliases and vector organization follow the public CMSIS
 * Core/Template/Device_M/Source/startup_Device.c template. This loader keeps
 * its existing Boot_Loader entry and C initialization. No CMSIS Reset_Handler
 * or __PROGRAM_START sequence is imported. Reserved slots retain a stop entry
 * for compatibility with the existing loader table.
 */

#include "arch.h"
#include "boot_vectors.h"

#define BOOT_FLASH __attribute__((section(".boot_text_flash.boot_vectors")))

__ALIGNED(BOOT_VECTOR_ALIGNMENT)
volatile uintptr_t boot_vector_table[BOOT_VECTOR_COUNT]
	__attribute__((section(".vector_table"), used));

BOOT_FLASH __attribute__((noreturn)) void boot_default_handler(void)
{
	for (;;) {
		__NOP();
	}
}

#define DEFAULT_HANDLER(name)                                                                      \
	void name(void) __attribute__((weak, alias("boot_default_handler"), noreturn))

DEFAULT_HANDLER(Reset_Handler);
DEFAULT_HANDLER(NMI_Handler);
DEFAULT_HANDLER(HardFault_Handler);
DEFAULT_HANDLER(MemManage_Handler);
DEFAULT_HANDLER(BusFault_Handler);
DEFAULT_HANDLER(UsageFault_Handler);
DEFAULT_HANDLER(SecureFault_Handler);
DEFAULT_HANDLER(SVC_Handler);
DEFAULT_HANDLER(DebugMon_Handler);
DEFAULT_HANDLER(PendSV_Handler);
DEFAULT_HANDLER(SysTick_Handler);

static const uintptr_t core_vectors[BOOT_CORE_VECTORS]
	__attribute__((section(".boot_rodata_flash.boot_vectors"))) = {
		(uintptr_t)&__StackTop,          (uintptr_t)Reset_Handler,
		(uintptr_t)NMI_Handler,          (uintptr_t)HardFault_Handler,
		(uintptr_t)MemManage_Handler,    (uintptr_t)BusFault_Handler,
		(uintptr_t)UsageFault_Handler,   (uintptr_t)SecureFault_Handler,
		(uintptr_t)boot_default_handler, (uintptr_t)boot_default_handler,
		(uintptr_t)boot_default_handler, (uintptr_t)SVC_Handler,
		(uintptr_t)DebugMon_Handler,     (uintptr_t)boot_default_handler,
		(uintptr_t)PendSV_Handler,       (uintptr_t)SysTick_Handler,
};

BOOT_FLASH void NVIC_InitVectors(void)
{
	/* Before SystemInit/BootInit: only ROM constants and this explicit RAM
	 * table may be accessed. No initialized globals, runtime or SRAM code.
	 */
	for (unsigned int i = 0U; i < BOOT_VECTOR_COUNT; i++) {
		boot_vector_table[i] =
			i < BOOT_CORE_VECTORS ? core_vectors[i] : (uintptr_t)boot_default_handler;
	}
	SCB->VTOR = (uintptr_t)boot_vector_table;
	__DSB();
	__ISB();
}

int NVIC_GetCurrentActiveIRQ(void)
{
	/* Existing trace ABI: Thread mode is -16, exceptions use IRQn numbering.
	 * All current loader entries fit in the signed-byte return convention.
	 */
	return (int8_t)(__get_IPSR() - BOOT_CORE_VECTORS);
}
