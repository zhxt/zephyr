# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Check current loader structure without SDK objects or historical fixtures."""

import io
import struct
import zlib

from elftools.elf.elffile import ELFFile


def loader_symbols(path):
    elf = ELFFile(io.BytesIO(path.read_bytes()))
    if elf.elfclass != 32 or not elf.little_endian or elf["e_machine"] != "EM_ARM":
        raise ValueError("Expected ELF32 little-endian ARM loader")
    table = elf.get_section_by_name(".symtab")
    if table is None:
        raise ValueError("Missing loader symbols")
    symbols = {s.name: int(s["st_value"]) for s in table.iter_symbols() if s.name}
    unresolved = [
        s.name
        for s in table.iter_symbols()
        if s.name and s["st_shndx"] == "SHN_UNDEF" and s["st_info"]["bind"] != "STB_WEAK"
    ]
    if unresolved:
        raise ValueError("Unresolved loader symbols: " + ",".join(unresolved))
    return elf, symbols


def validate(path, image, payload, layout, required):
    elf, symbols = loader_symbols(path)
    missing = set(required) - set(symbols)
    if missing:
        raise ValueError("Missing required hardware interfaces: " + ",".join(sorted(missing)))
    sizes = {
        ".boot_struct": 16,
        ".vector_table": 320,
        ".reboot_param": 64,
        ".got_info": 0,
        ".sram_data": 0,
        ".fast_text_sram": 0,
    }
    for name, size in sizes.items():
        section = elf.get_section_by_name(name)
        if (section is None and size != 0) or (
            section is not None and int(section["sh_size"]) != size
        ):
            raise ValueError("Invalid loader section geometry: " + name)
    for name in (".boot_text_sram", ".boot_data_sram", ".boot_bss_sram"):
        section = elf.get_section_by_name(name)
        address, size = int(section["sh_addr"]), int(section["sh_size"])
        base = 0x00500180 if name == ".boot_text_sram" else 0x20500180
        top = 0x0050F000 if name == ".boot_text_sram" else 0x2050F000
        if not base <= address <= top or size > top - address:
            raise ValueError("Early SRAM range is outside loader reservation")
    if symbols["__StackTop"] != 0x2050F000 or symbols["__StackLimit"] != 0x2050DF00:
        raise ValueError("Loader stack contract changed")
    start, end = symbols["bth_payload_start"], symbols["bth_payload_end"]
    found = []
    for segment in elf.iter_segments():
        if segment["p_type"] != "PT_LOAD" or segment["p_filesz"] == 0:
            continue
        vaddr, size = int(segment["p_vaddr"]), int(segment["p_filesz"])
        paddr = int(segment["p_paddr"])
        if not 0x34000000 <= paddr < 0x347FB000 or size > 0x347FB000 - paddr:
            raise ValueError("Loader load range overlaps reserved flash")
        if vaddr <= start and end <= vaddr + size:
            found.append((paddr - 0x34000000 + start - vaddr, end - start))
    if len(found) != 1 or found[0][1] != len(payload):
        raise ValueError("Payload is not uniquely file-backed in loader")
    offset = found[0][0]
    if image[offset : offset + len(payload)] != payload:
        raise ValueError("ROM payload differs from checked application")
    fields = struct.unpack_from("<16I", payload)
    if (fields[0], fields[1], fields[2], fields[3], fields[4]) != (
        0x35485442,
        1,
        0x00050001,
        int(layout["build"], 16),
        len(payload) - 64,
    ):
        raise ValueError("Payload identity or length changed")
    if zlib.crc32(payload[64:]) != fields[5]:
        raise ValueError("Payload CRC mismatch")
    if struct.unpack_from("<3I", image) != (0xFFFFFFFF, 0x00040000, 0):
        raise ValueError("ROM header changed")
    if struct.unpack_from("<I", image, len(image) - 4)[0] != 0x34000000:
        raise ValueError("ROM trailer changed")
    return {
        "status": "pass_for_production_image_structure",
        "required_interfaces_present": len(required),
        "payload_offset": offset,
        "payload_bytes": len(payload),
        "ROM_header_and_trailer_verified": True,
        "payload_CRC_verified": True,
        "reserved_ranges_and_stack_verified": True,
        "hardware_validated": False,
        "public_BES_hardware_contract_established": False,
    }
