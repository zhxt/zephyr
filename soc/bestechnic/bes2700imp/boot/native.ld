/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

ENTRY(bes2700imp_rom_entry)

SECTIONS
{
	/* NOR image offsets are relative to the instruction alias in the raw file. */
	.boot_header 0x0c000000 : { KEEP(*(.boot_header)) }
	.text : {
		KEEP(*(.text.native_entry))
		*(.text .text.*)
	}
	/* Reserve space for the relocated routines at the base of the loader stack. */
	.ramtext 0x200b9000 : AT(ALIGN(LOADADDR(.text) + SIZEOF(.text), 4)) {
		__ramtext_start = .;
		*(.ramtext)
		. = ALIGN(4);
		__ramtext_end = .;
	}
	.rodata 0x2c001000 : AT(0x0c001000) {
		KEEP(*(.build_info))
		*(.rodata .rodata.*)
	}

	__build_info_physical = ADDR(.rodata) - 0x04000000;
	__fault_vectors = 0x200b2000;
	/* Read the routines through the NOR data alias. */
	__ramtext_load = LOADADDR(.ramtext) + ADDR(.rodata) - LOADADDR(.rodata);
	__stack_bottom = ALIGN(__ramtext_end, 8);
	__stack_top = 0x200ba000;

	/DISCARD/ : { *(.ARM.exidx* .ARM.extab* .comment .note*) }
}

ASSERT(SIZEOF(.boot_header) == 16, "invalid NOR header length")
ASSERT(bes2700imp_rom_entry == 0x0c000010, "entry must immediately follow NOR header")
ASSERT(LOADADDR(.ramtext) + SIZEOF(.ramtext) <= LOADADDR(.rodata),
       "loader code overlaps constants")
ASSERT(SIZEOF(.ramtext) > 0 && SIZEOF(.ramtext) <= 0x100,
       "loader SRAM code must fit while leaving 3840 bytes of stack")
