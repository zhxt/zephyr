/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

.syntax unified
.cpu cortex-m33
.thumb

.section .boot_header,"a",%progbits
.word 0xbe57ec1c
.hword 0, 5
.word 0
.word __build_info_physical

.section .text.native_entry,"ax",%progbits
.global bes2700imp_rom_entry
.type bes2700imp_rom_entry,%function
.thumb_func
bes2700imp_rom_entry:
	/* Normalize the instruction alias before using PC-relative addresses. */
	ldr r0, =bes2700imp_rom_start
	bx r0
.size bes2700imp_rom_entry, . - bes2700imp_rom_entry
.type bes2700imp_rom_start,%function
.thumb_func
bes2700imp_rom_start:
	cpsid i
	movs r0, #0
	msr msplim, r0
	ldr r1, =__stack_top
	msr msp, r1
	msr control, r0
	isb
	ldr r1, =__stack_bottom
	msr msplim, r1
	msr basepri, r0
	dsb
	isb
	bl bes2700imp_rom_main
	udf #0
.size bes2700imp_rom_start, . - bes2700imp_rom_start
