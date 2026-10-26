/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Experimental loader entry. Hardware preparation remains in the vendor HAL.
 * The M55 helper uses LR as the BTH entry, so its next instruction must be the
 * start of the BTH stack setup. No runtime storage or copied SRAM code is used
 * before the selected SRAM initialization has prepared it.
 */
.syntax unified
.thumb

.section .boot_loader, "ax", %progbits
.balign 4
.global Boot_Loader
.type Boot_Loader, %function
.thumb_func
Boot_Loader:
	ldr r0, =boot_entry_m55
	bx r0

.global boot_entry_m55
.type boot_entry_m55, %function
.thumb_func
boot_entry_m55:
	bl hal_cmu_boot_m55_start_bth
.size boot_entry_m55, . - boot_entry_m55

.global boot_entry_bth
.type boot_entry_bth, %function
.thumb_func
boot_entry_bth:
	ldr r0, =__StackTop
	msr msp, r0
	ldr r0, =__StackLimit
	msr msplim, r0
	movs r0, #0
	msr control, r0
	isb
	bl NVIC_InitVectors
#ifdef BES_BOOT_RUNTIME_SPLIT
	bl boot_cpu_init
	bl bes_boot_early_cache_prepare
	ldr r9, =__got_info_start
	bl boot_crt_copy_boot_sram
	bl bes_boot_early_clock_prepare
	bl boot_crt_copy_other_sram
	bl boot_crt_init
#else
	bl SystemInit
	bl BootInit

	/* The retained linker script supplies word-aligned data and BSS ranges. */
	ldr r0, =__etext
	ldr r1, =__data_start__
	ldr r2, =__data_end__
.Lcopy:
	cmp r1, r2
	bhs .Lzero_start
	ldr r3, [r0], #4
	str r3, [r1], #4
	b .Lcopy
.Lzero_start:
	ldr r1, =__bss_start__
	ldr r2, =__bss_end__
	movs r0, #0
.Lzero:
	cmp r1, r2
	bhs .Ladapter
	str r0, [r1], #4
	b .Lzero
#endif
.Ladapter:
	bl bth_boot_adapter_main

.global boot_entry_returned
.type boot_entry_returned, %function
.thumb_func
boot_entry_returned:
	b boot_entry_returned
.size boot_entry_returned, . - boot_entry_returned
	.balign 4
	.ltorg
.size boot_entry_bth, . - boot_entry_bth
.size Boot_Loader, . - Boot_Loader

.section .note.GNU-stack, "", %progbits
