/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Experimental layout written from the observed ELF/image ABI. Addresses are
 * aliases of the same Flash or RAM; no general ROM specification is implied.
 * Empty overlay slots and the reboot reservation remain ABI compatibility data.
 */
OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(Boot_Loader)

SECTIONS
{
	.boot_struct 0x34000000 : { KEEP(*(.boot_struct)) }
	.boot_text_flash 0x14000010 : AT(0x34000010) {
		KEEP(*(.boot_loader))
		*(.boot_text_flash .boot_text_flash.*)
		*(.boot_rodata_flash .boot_rodata_flash.*)
		. = ALIGN(4);
	}
	.got_info (LOADADDR(.boot_text_flash) + SIZEOF(.boot_text_flash)) : {
		__got_info_start = .;
		__got_start = .; *(.got) __got_end = .;
		__got_plt_start = .; *(.got.plt) __got_plt_end = .;
	}
	.vector_table 0x20500000 (NOLOAD) : AT(ADDR(.vector_table)) ALIGN(1024) {
		__vector_table_start = .;
		KEEP(*(.vector_table))
	}
	.reboot_param 0x20500140 (NOLOAD) : AT(ADDR(.reboot_param)) {
		KEEP(*(.reboot_param))
		. = 64;
	}
	.userdata_pool 0x20500180 (NOLOAD) : AT(ADDR(.userdata_pool)) { __userdata_pool_end__ = .; }
	.boot_text_sram 0x00500180 : AT(ADDR(.got_info) + SIZEOF(.got_info)) {
		/* Flash routines must run while their own storage is unavailable. */
		*(.boot_text_sram .boot_text_sram.*)
		. = ALIGN(4);
	}
	.boot_data_sram (ADDR(.boot_text_sram) + SIZEOF(.boot_text_sram) + 0x20000000) :
		AT(LOADADDR(.boot_text_sram) + SIZEOF(.boot_text_sram)) {
		*(.boot_data .boot_data.*)
		. = ALIGN(4);
	}
	.boot_bss_sram (ADDR(.boot_data_sram) + SIZEOF(.boot_data_sram)) (NOLOAD) :
		AT(ADDR(.boot_bss_sram)) {
		*(.boot_bss .boot_bss.*)
		. = ALIGN(4);
	}
	.sram_text (ALIGN(ADDR(.boot_bss_sram) + SIZEOF(.boot_bss_sram), 32) - 0x20000000) :
		AT(LOADADDR(.boot_data_sram) + SIZEOF(.boot_data_sram)) {
		*(.sram_text .sram_text.*)
		. = ALIGN(32);
	}
	.sram_data (ADDR(.sram_text) + SIZEOF(.sram_text) + 0x20000000) :
		AT(LOADADDR(.sram_text) + SIZEOF(.sram_text)) {
		*(.sram_data .sram_data.*)
		_hdf_drivers_start = .; _hdf_drivers_end = .;
	}
	.sram_bss (ADDR(.sram_data) + SIZEOF(.sram_data)) (NOLOAD) : AT(ADDR(.sram_bss)) {
		*(.sram_bss .sram_bss.*) . = ALIGN(32);
	}
	.fast_text_sram 0x0050f000 : AT(LOADADDR(.sram_data) + SIZEOF(.sram_data)) {
		*(.fast_text_sram .fast_text_sram.*)
	}
	OVERLAY 0x0050f000 : AT(LOADADDR(.fast_text_sram) + SIZEOF(.fast_text_sram)) {
		.overlay_text0 { LONG(0) }
		.overlay_text1 { LONG(0) }
		.overlay_text2 { LONG(0) }
		.overlay_text3 { LONG(0) }
		.overlay_text4 { LONG(0) }
		.overlay_text5 { LONG(0) }
		.overlay_text6 { LONG(0) }
		.overlay_text7 { LONG(0) }
		.overlay_text8 { LONG(0) }
		.overlay_text9 { LONG(0) }
		.overlay_text10 { LONG(0) }
		.overlay_text11 { LONG(0) }
		.overlay_text_last { LONG(0) }
	}
	OVERLAY (ADDR(.sram_bss) + SIZEOF(.sram_bss)) :
		AT(LOADADDR(.overlay_text_last) + SIZEOF(.overlay_text_last)) {
		.overlay_data0 { LONG(0) }
		.overlay_data1 { LONG(0) }
		.overlay_data2 { LONG(0) }
		.overlay_data3 { LONG(0) }
		.overlay_data4 { LONG(0) }
		.overlay_data5 { LONG(0) }
		.overlay_data6 { LONG(0) }
		.overlay_data7 { LONG(0) }
		.overlay_data8 { LONG(0) }
		.overlay_data9 { LONG(0) }
		.overlay_data10 { LONG(0) }
		.overlay_data11 { LONG(0) }
		.overlay_data_last { LONG(0) }
	}
	.text (ALIGN(LOADADDR(.overlay_data_last) + SIZEOF(.overlay_data_last), 8) - 0x20000000) :
		AT(ALIGN(LOADADDR(.overlay_data_last) + SIZEOF(.overlay_data_last), 8)) ALIGN(8) {
		*(.text .text.*)
		. = ALIGN(8);
	}
	.ARM.extab : AT(LOADADDR(.text) + SIZEOF(.text)) { *(.ARM.extab*) }
	.ARM.exidx : AT(LOADADDR(.text) + SIZEOF(.text) + SIZEOF(.ARM.extab)) {
		__exidx_start = .; *(.ARM.exidx*) __exidx_end = .;
	}
	.rodata (LOADADDR(.ARM.exidx) + SIZEOF(.ARM.exidx)) :
		AT(LOADADDR(.ARM.exidx) + SIZEOF(.ARM.exidx)) {
		*(.rodata .rodata.*)
		*(.flash_rodata .flash_rodata.*)
		. = ALIGN(4);
		/* Empty registration tables share the start of the build-id note. */
		*(.note.gnu.build-id)
	}
	.zInit (ADDR(.rodata) + SIZEOF(.rodata)) : { }
	.data (ADDR(.overlay_data_last) + SIZEOF(.overlay_data_last)) :
		AT(ADDR(.rodata) + SIZEOF(.rodata)) {
		*(.data .data.*)
		. = ALIGN(4);
	}
	.bss (ADDR(.data) + SIZEOF(.data)) (NOLOAD) :
		AT(ADDR(.bss)) { *(.bss .bss.* COMMON) . = ALIGN(4); }
	.framebuffer (NOLOAD) : AT(ADDR(.framebuffer)) { . = ALIGN(32); }
	.reboot_custom_param (NOLOAD) : AT(ADDR(.reboot_custom_param)) { __reboot_custom_param_end__ = .; }
	.heap (NOLOAD) : AT(ADDR(.heap)) { . += 0x1000; }
	.stack_dummy (ADDR(.heap) + SIZEOF(.heap)) (COPY) : { . += 0x1100; }
	.build_info (LOADADDR(.data) + SIZEOF(.data)) :
		AT(LOADADDR(.data) + SIZEOF(.data)) { KEEP(*(.build_info)) . = ALIGN(4); }
	.code_start_addr : { LONG(0) LONG(0xbe57341d) LONG(0x34000000) }
	.custom_parameter 0x307fb000 (NOLOAD) : AT(ADDR(.custom_parameter)) { . += 0x1000; }
	.userdata 0x307fc000 (NOLOAD) : AT(ADDR(.userdata)) { . += 0x2000; }
	.reserved 0x307fe000 (NOLOAD) : AT(ADDR(.reserved)) { . += 0x1000; }
	.factory 0x307ff000 (NOLOAD) : AT(ADDR(.factory)) { . += 0x1000; }
	/DISCARD/ : { *(.note.GNU-stack) }

	/* Retained ABI names, expressed through the regions that own them. */
	__HeapBase = ADDR(.heap);
	__HeapLimit = ADDR(.heap) + SIZEOF(.heap);
	__StackLimit = __StackTop - SIZEOF(.stack_dummy);
	__StackTop = 0x2050f000;
	__ai_ble_handler_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__ai_ble_handler_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__ai_gatt_server_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__ai_gatt_server_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__ai_handler_function_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__ai_handler_function_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__ai_spp_register_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__ai_spp_register_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__app_bt_sync_command_handler_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__app_bt_sync_command_handler_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__aud_end = ADDR(.reserved);
	__aud_start = ADDR(.reserved);
	__boot_bss_sram_end__ = ADDR(.boot_bss_sram) + SIZEOF(.boot_bss_sram);
	__boot_bss_sram_start__ = ADDR(.boot_bss_sram);
	__boot_data_sram_end__ = ADDR(.boot_data_sram) + SIZEOF(.boot_data_sram);
	__boot_data_sram_start__ = ADDR(.boot_data_sram);
	__boot_data_sram_start_flash__ = LOADADDR(.boot_data_sram);
	__boot_sram_end_flash__ = LOADADDR(.boot_data_sram) + SIZEOF(.boot_data_sram);
	__boot_sram_start__ = ADDR(.boot_text_sram) + 0x20000000;
	__boot_sram_start_flash__ = LOADADDR(.boot_text_sram);
	__boot_text_sram_end__ = ADDR(.boot_text_sram) + SIZEOF(.boot_text_sram);
	__boot_text_sram_end_flash__ = LOADADDR(.boot_data_sram);
	__boot_text_sram_start__ = ADDR(.boot_text_sram);
	__bss_end__ = ADDR(.bss) + SIZEOF(.bss);
	__bss_start__ = ADDR(.bss);
	__bt_controller_ramdump_section_end = ADDR(.custom_parameter);
	__bt_controller_ramdump_section_start = ADDR(.custom_parameter);
	__combo_end = ADDR(.custom_parameter);
	__combo_start = ADDR(.custom_parameter);
	__core_bridge_instant_cmd_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__core_bridge_instant_cmd_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__core_bridge_task_cmd_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__core_bridge_task_cmd_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__coredump_section_end = ADDR(.custom_parameter);
	__coredump_section_start = ADDR(.custom_parameter);
	__crash_dump_end = ADDR(.custom_parameter);
	__crash_dump_start = ADDR(.custom_parameter);
	__cust_cmd_hldr_tbl_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__cust_cmd_hldr_tbl_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__custom_handler_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__custom_handler_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__custom_parameter_end = ADDR(.custom_parameter) + SIZEOF(.custom_parameter);
	__custom_parameter_start = ADDR(.custom_parameter);
	__data_end__ = ADDR(.data) + SIZEOF(.data);
	__data_start__ = ADDR(.data);
	__end__ = ADDR(.heap);
	__eshell_command_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__eshell_command_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__etext = LOADADDR(.data);
	__export_fn_rom = 0x37f58;
	__factory_end = ADDR(.factory) + SIZEOF(.factory);
	__factory_start = ADDR(.factory);
	__fast_sram_end__ = __overlay_text_start__ + SIZEOF(.overlay_text_last);
	__fast_sram_text_data_end__ = ADDR(.fast_text_sram) + 0x20000000;
	__fast_sram_text_data_end_flash__ = LOADADDR(.overlay_text0);
	__fast_sram_text_data_start__ = ADDR(.fast_text_sram) + 0x20000000;
	__fast_sram_text_data_start_flash__ = LOADADDR(.fast_text_sram);
	__fast_sram_text_exec_end__ = ADDR(.fast_text_sram);
	__fast_sram_text_exec_start__ = ADDR(.fast_text_sram);
	__flash_end = ADDR(.code_start_addr) + SIZEOF(.code_start_addr);
	__flash_start = ADDR(.boot_struct);
	__flash_text_data_start__ = LOADADDR(.text);
	__flashx_start_addr__ = ADDR(.boot_text_flash);
	__flashx_text_end__ = ADDR(.ARM.exidx) + SIZEOF(.ARM.exidx);
	__free_flash = 0x347fb000 - __flash_end;
	__free_fram = 0x1000 - SIZEOF(.overlay_text_last);
	__free_ram = __StackLimit - __HeapLimit;
	__hotword_model_end = ADDR(.custom_parameter);
	__hotword_model_start = ADDR(.custom_parameter);
	__lhdc_license_end = ADDR(.custom_parameter);
	__lhdc_license_start = ADDR(.custom_parameter);
	__load_stop_overlay_data_last = LOADADDR(.overlay_data_last) + SIZEOF(.overlay_data_last);
	__load_stop_overlay_text_last = LOADADDR(.overlay_text_last) + SIZEOF(.overlay_text_last);
	__log_dump_end = ADDR(.custom_parameter);
	__log_dump_start = ADDR(.custom_parameter);
	__mem_pool_end__ = __StackLimit;
	__mem_pool_start__ = __HeapLimit;
	__mixprompt_property_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__mixprompt_property_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__ota_upgrade_log_end = ADDR(.custom_parameter);
	__ota_upgrade_log_start = ADDR(.custom_parameter);
	__overlay_data_end__ = ADDR(.overlay_data_last) + SIZEOF(.overlay_data_last);
	__overlay_data_start__ = ADDR(.overlay_data_last);
	__overlay_text_exec_end__ = ADDR(.overlay_text_last) + SIZEOF(.overlay_text_last);
	__overlay_text_exec_start__ = ADDR(.overlay_text_last);
	__overlay_text_start__ = ADDR(.fast_text_sram) + 0x20000000;
	__overlay_text_start_flash__ = LOADADDR(.overlay_text0);
	__prompt_end = ADDR(.custom_parameter);
	__prompt_start = ADDR(.custom_parameter);
	__ramdump_section_end = ADDR(.custom_parameter);
	__ramdump_section_start = ADDR(.custom_parameter);
	__reserved_end = ADDR(.reserved) + SIZEOF(.reserved);
	__reserved_start = ADDR(.reserved);
	__rpc_bth_dsp_cmd_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__rpc_bth_dsp_cmd_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__shell_root_cmds_end = ADDR(.data) + SIZEOF(.data);
	__shell_root_cmds_start = ADDR(.data) + SIZEOF(.data);
	__sram_bss_end__ = ADDR(.sram_bss) + SIZEOF(.sram_bss);
	__sram_bss_start__ = ADDR(.sram_bss);
	__sram_data_end__ = ADDR(.sram_data) + SIZEOF(.sram_data);
	__sram_data_start__ = ADDR(.sram_data);
	__sram_data_start_flash__ = LOADADDR(.sram_data);
	__sram_text_data_end_flash__ = LOADADDR(.sram_data) + SIZEOF(.sram_data);
	__sram_text_data_start__ = ADDR(.sram_text) + 0x20000000;
	__sram_text_data_start_flash__ = LOADADDR(.sram_text);
	__sram_text_end__ = ADDR(.sram_text) + SIZEOF(.sram_text);
	__sram_text_end_flash__ = LOADADDR(.sram_data);
	__sram_text_start__ = ADDR(.sram_text);
	__tail_section_start = ADDR(.custom_parameter);
	__tota_handler_table_end = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__tota_handler_table_start = ADDR(.rodata) + SIZEOF(.rodata) - 36;
	__userdata_end = ADDR(.userdata) + SIZEOF(.userdata);
	__userdata_start = ADDR(.userdata);
	__zinitcall_app_feature_end = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_app_feature_start = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_app_service_end = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_app_service_start = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_bsp_end = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_bsp_start = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_core_end = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_core_start = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_device_end = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_device_start = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_exit_end = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_exit_start = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_run_end = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_run_start = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_sys_feature_end = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_sys_feature_start = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_sys_service_end = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_sys_service_start = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_test_end = ADDR(.rodata) + SIZEOF(.rodata);
	__zinitcall_test_start = ADDR(.rodata) + SIZEOF(.rodata);
	_ebss = ADDR(.bss) + SIZEOF(.bss);
	_edata = ADDR(.data) + SIZEOF(.data);
	_einit = ADDR(.ARM.exidx);
	_etext = LOADADDR(.data);
	_sbss = ADDR(.bss);
	_sdata = ADDR(.data);
	_sinit = ADDR(.ARM.exidx);
	_stext = ADDR(.boot_text_sram);
	end = ADDR(.heap);

	ASSERT(SIZEOF(.boot_struct) == 16, "boot header size")
	ASSERT(SIZEOF(.vector_table) == 320, "vector table size")
	ASSERT(SIZEOF(.reboot_param) == 64, "reboot reservation size")
	ASSERT(SIZEOF(.got_info) == 0, "unexpected GOT input")
	ASSERT(SIZEOF(.sram_data) == 0, "unexpected SRAM data input")
	ASSERT(ADDR(.boot_bss_sram) >= 0x20500180 &&
	       ADDR(.boot_bss_sram) + SIZEOF(.boot_bss_sram) <= __StackLimit,
	       "boot BSS outside data alias")
	ASSERT(ADDR(.bss) >= 0x20500180 && ADDR(.bss) + SIZEOF(.bss) <= __StackLimit,
	       "normal BSS outside data alias")
	ASSERT(SIZEOF(.fast_text_sram) == 0, "unexpected fast SRAM input")
	ASSERT(ADDR(.heap) + SIZEOF(.heap) <= 0x2050df00, "loader stack overlap")
	ASSERT(ADDR(.code_start_addr) + SIZEOF(.code_start_addr) <= 0x347fb000,
	       "loader image overlaps Flash tail")
}
