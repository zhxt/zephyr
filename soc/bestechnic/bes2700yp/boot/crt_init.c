/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

#include "arch.h"
#include "runtime.h"

extern uint32_t __boot_sram_start_flash__[], __boot_sram_end_flash__[], __boot_sram_start__[];
extern uint32_t __boot_bss_sram_start__[], __boot_bss_sram_end__[];
extern uint32_t __sram_text_data_start_flash__[], __sram_text_data_end_flash__[];
extern uint32_t __sram_text_data_start__[], __sram_bss_start__[], __sram_bss_end__[];
extern uint32_t __fast_sram_text_data_start_flash__[], __fast_sram_text_data_end_flash__[];
extern uint32_t __fast_sram_text_data_start__[];
extern uint32_t __etext[], __data_start__[], __data_end__[], __bss_start__[], __bss_end__[];

/* Volatile word accesses prevent conversion into calls to uninitialized SRAM
 * libc. The build audit requires aligned, disjoint linker ranges. Integer
 * addresses avoid comparisons between distinct linker-declared C arrays.
 */
static __attribute__((always_inline)) inline void copy_words(uintptr_t destination,
							     uintptr_t source, uintptr_t source_end)
{
	while (source < source_end) {
		*(volatile uint32_t *)destination = *(const volatile uint32_t *)source;
		destination += sizeof(uint32_t);
		source += sizeof(uint32_t);
	}
}

static __attribute__((always_inline)) inline void zero_words(uintptr_t destination, uintptr_t end)
{
	while (destination < end) {
		*(volatile uint32_t *)destination = 0U;
		destination += sizeof(uint32_t);
	}
}

__attribute__((section(".boot_text_flash.crt_boot_sram"))) void boot_crt_copy_boot_sram(void)
{
	copy_words((uintptr_t)__boot_sram_start__, (uintptr_t)__boot_sram_start_flash__,
		   (uintptr_t)__boot_sram_end_flash__);
	__DSB();
	__ISB();
	zero_words((uintptr_t)__boot_bss_sram_start__, (uintptr_t)__boot_bss_sram_end__);
}

__attribute__((section(".boot_text_flash.crt_other_sram"))) void boot_crt_copy_other_sram(void)
{
	copy_words((uintptr_t)__sram_text_data_start__, (uintptr_t)__sram_text_data_start_flash__,
		   (uintptr_t)__sram_text_data_end_flash__);
	copy_words((uintptr_t)__fast_sram_text_data_start__,
		   (uintptr_t)__fast_sram_text_data_start_flash__,
		   (uintptr_t)__fast_sram_text_data_end_flash__);
	__DSB();
	__ISB();
	zero_words((uintptr_t)__sram_bss_start__, (uintptr_t)__sram_bss_end__);
}

__attribute__((section(".boot_text_flash.crt_normal"))) void boot_crt_init(void)
{
	copy_words((uintptr_t)__data_start__, (uintptr_t)__etext,
		   (uintptr_t)__etext + (uintptr_t)__data_end__ - (uintptr_t)__data_start__);
	zero_words((uintptr_t)__bss_start__, (uintptr_t)__bss_end__);
}
