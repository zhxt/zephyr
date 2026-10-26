/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

/* Keep the existing cache/CMU calling ABI isolated from generic CPU/CRT code.
 * Cache arguments 0, 1, 1, 1 describe this pinned vendor experiment only.
 * The wrappers need neither initialized data nor copied SRAM instructions.
 */
.syntax unified
.thumb

.section .boot_text_flash.early_vendor_cache, "ax", %progbits
.balign 4
.global bes_boot_early_cache_prepare
.type bes_boot_early_cache_prepare, %function
.thumb_func
bes_boot_early_cache_prepare:
	push {r4, lr}
	movs r0, #0
	bl hal_cache_enable
	movs r0, #1
	bl hal_cache_enable
	movs r0, #1
	bl hal_cache_writebuffer_enable
	movs r0, #1
	bl hal_cache_writeback_enable
	pop {r4, pc}
.size bes_boot_early_cache_prepare, . - bes_boot_early_cache_prepare

.section .boot_text_flash.early_vendor_clock, "ax", %progbits
.balign 4
.global bes_boot_early_clock_prepare
.type bes_boot_early_clock_prepare, %function
.thumb_func
bes_boot_early_clock_prepare:
	b.w hal_cmu_setup
.size bes_boot_early_clock_prepare, . - bes_boot_early_clock_prepare

.section .note.GNU-stack, "", %progbits
