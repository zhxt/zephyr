#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

"""Build the BES2700iMP v5 ROM image for a Zephyr SRAM payload."""

import argparse
import hashlib
import json
import struct
import subprocess
import tempfile
from pathlib import Path

from elftools.elf.elffile import ELFFile

FLASH_START = 0x28000000
FLASH_SIZE = 128 * 1024
FLASH_ALIAS = 0x0C000000
FLASH_DATA_ALIAS = 0x2C000000
HEADER = struct.pack("<IHHII", 0xBE57EC1C, 0, 5, 0, FLASH_START + 0x1000)
BOARD_LAYOUTS = {
    (0x20070000, 0xF000, 24_000_000),
    (0x20010000, 0x24000, 48_000_000),
}
SOURCES = ("native_entry.S", "native.c", "payload.S", "native.ld")
BUILD_ID_PLACEHOLDER = b"0000000000000000"
BUILD_ID_PREFIX = b"BES2700IMP_ROM build="
BUILD_ID_SUFFIX = b" stage=entry\r\n"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def symbol(elf, name):
    symbols = elf.get_section_by_name(".symtab").get_symbol_by_name(name)
    require(symbols and len(symbols) == 1, f"missing or ambiguous ELF symbol {name}")
    return symbols[0]["st_value"]


def verify_payload(path, payload, ram_base, ram_size):
    require(len(payload) >= 16 * 4, "truncated Zephyr vector table")
    stack, reset = struct.unpack_from("<II", payload)
    pend_sv, systick = struct.unpack_from("<II", payload, 14 * 4)
    require(ram_base < stack <= ram_base + ram_size and stack % 8 == 0, "invalid initial MSP")
    with path.open("rb") as file:
        elf = ELFFile(file)
        require(elf["e_type"] == "ET_EXEC" and elf["e_machine"] == "EM_ARM", "invalid Zephyr ELF")
        require(symbol(elf, "_vector_table") == ram_base, "unexpected Zephyr vector table address")
        require(reset == elf["e_entry"] == symbol(elf, "z_arm_reset"), "incorrect reset vector")
        require(
            all(address & 1 and ram_base <= address - 1 < ram_base + len(payload)
                for address in (reset, pend_sv, systick)),
            "invalid reset, PendSV or SysTick Thumb address",
        )
        segments = [segment for segment in elf.iter_segments() if segment["p_type"] == "PT_LOAD"]
        require(len(segments) == 1, "expected one contiguous Zephyr load")
        load = segments[0]
        require(
            load["p_paddr"] == load["p_vaddr"] == ram_base
            and len(payload) == load["p_filesz"] <= load["p_memsz"] <= ram_size,
            "Zephyr load exceeds the board SRAM window",
        )
        for section in elf.iter_sections():
            if (
                section["sh_flags"] & 2
                and section["sh_type"] != "SHT_NOBITS"
                and section["sh_size"]
                and load.section_in_segment(section)
            ):
                offset = section["sh_addr"] - ram_base
                require(payload[offset:offset + section["sh_size"]] == section.data(),
                        f"incorrect Zephyr binary section {section.name}")
        return dict(vector_table=ram_base, stack=stack, reset=reset, pend_sv=pend_sv,
                    systick=systick, memory_bytes=load["p_memsz"], payload_bytes=len(payload),
                    payload_sha256=sha256(payload))


def verify_loader(path, raw, payload):
    with path.open("rb") as file:
        elf = ELFFile(file)
        require(elf["e_type"] == "ET_EXEC" and elf["e_machine"] == "EM_ARM", "invalid loader ELF")
        entry = symbol(elf, "bes2700imp_rom_entry")
        require(entry == elf["e_entry"] == FLASH_ALIAS + 0x11, "incorrect loader entry")
        require(
            all(segment["p_type"] not in ("PT_INTERP", "PT_DYNAMIC")
                for segment in elf.iter_segments()),
            "loader requires a dynamic runtime",
        )
        require(
            not any(sym.name and sym["st_shndx"] == "SHN_UNDEF"
                    for sym in elf.get_section_by_name(".symtab").iter_symbols()),
            "loader has undefined symbols",
        )
        sections = {sec.name: sec for sec in elf.iter_sections()
                    if sec["sh_flags"] & 2 and sec["sh_size"]}
        require(sections.keys() == {".boot_header", ".text", ".ramtext", ".rodata"},
                "unexpected loader sections")
        flash_text_end = sections[".text"]["sh_addr"] + sections[".text"]["sh_size"]
        for section in sections.values():
            offset = section["sh_addr"] & 0x00FFFFFF
            if section.name == ".ramtext":
                start = section["sh_addr"]
                end = start + section["sh_size"]
                loads = [segment for segment in elf.iter_segments()
                         if segment["p_type"] == "PT_LOAD" and segment.section_in_segment(section)]
                require(len(loads) == 1, "expected one load segment for loader SRAM code")
                load = loads[0]["p_paddr"] + start - loads[0]["p_vaddr"]
                offset = load - FLASH_ALIAS
                entries = [symbol(elf, name) for name in ("copy_payload", "payload_digest")]
                require(
                    load >= flash_text_end
                    and offset + section["sh_size"] <= 0x1000
                    and symbol(elf, "__ramtext_load") == FLASH_DATA_ALIAS + offset
                    and start == symbol(elf, "__ramtext_start") == 0x200B9000
                    and end == symbol(elf, "__ramtext_end") <= 0x200B9100
                    and end <= symbol(elf, "__stack_bottom") <= 0x200B9100
                    and section["sh_flags"] & 4
                    and all(entry & 1 and start <= entry - 1 < end for entry in entries),
                    "invalid loader SRAM code placement",
                )
            require(raw[offset:offset + section["sh_size"]] == section.data(),
                    f"incorrect loader binary section {section.name}")
        require(raw.startswith(HEADER), "incorrect v5 ROM header")
        literal, branch = struct.unpack_from("<HH", raw, 16)
        require(literal & 0xFF00 == 0x4800 and branch == 0x4700, "missing ROM alias branch")
        branch_target, = struct.unpack_from("<I", raw, 20 + 4 * (literal & 0xFF))
        require(branch_target & 1 and branch_target == symbol(elf, "bes2700imp_rom_start"),
                "incorrect ROM alias branch destination")
        require(raw[0x1000:].startswith(b"CHIP=best1503\nSW_TYPE=ZEPHYR_BES2700IMP\n\0"),
                "incorrect ROM boot information")
        start = symbol(elf, "payload_start")
        end = symbol(elf, "payload_end")
        offset = start - FLASH_DATA_ALIAS
        require(offset >= 0 and end - start == len(payload)
                and raw[offset:offset + len(payload)] == payload, "incorrect embedded Zephyr binary")
        return dict(rom_entry=entry, rom_branch=branch_target, embedded_offset=offset)


def set_build_id(path, image, payload_offset):
    # Hash the complete flash image with a fixed value in its own ID field.
    marker = BUILD_ID_PREFIX + BUILD_ID_PLACEHOLDER + BUILD_ID_SUFFIX
    identity = sha256(image)[:len(BUILD_ID_PLACEHOLDER)]
    with path.open("r+b") as file:
        rodata = ELFFile(file).get_section_by_name(".rodata")
        section_start = rodata["sh_addr"] & 0x00FFFFFF
        constants_size = payload_offset - section_start
        data = rodata.data()
        require(0 <= constants_size <= len(data), "incorrect loader constant range")
        constants = data[:constants_size]
        require(constants.count(marker) == 1, "missing or ambiguous loader build ID")
        field = constants.index(marker) + len(BUILD_ID_PREFIX)
        file.seek(rodata["sh_offset"] + field)
        file.write(identity.encode("ascii"))

    start = section_start + field
    end = start + len(BUILD_ID_PLACEHOLDER)
    return identity, image[:start] + identity.encode("ascii") + image[end:]


def generate(args):
    layout = (args.ram_base, args.ram_size, args.clock_hz)
    require(layout in BOARD_LAYOUTS, "unsupported board SRAM and clock combination")
    inputs = [args.boot / name for name in SOURCES] + [args.handoff]
    sources = {path.name: sha256(path.read_bytes()) for path in inputs}
    with tempfile.TemporaryDirectory(prefix="bes2700imp-", dir=args.output.parent) as temp:
        work = Path(temp)
        payload_path = work / "payload.bin"
        subprocess.run([args.payload_objcopy, "-O", "binary", "--gap-fill", "0xff",
                        args.payload, payload_path], check=True)
        payload = payload_path.read_bytes()
        application = verify_payload(args.payload, payload, args.ram_base, args.ram_size)
        digest = 2166136261
        for byte in payload:
            digest = ((digest ^ byte) * 16777619) & 0xFFFFFFFF

        (work / "image_config.h").write_text(
            f'#define NATIVE_BUILD "{BUILD_ID_PLACEHOLDER.decode("ascii")}"\n'
            f"#define PAYLOAD_FILE {json.dumps(str(payload_path))}\n"
            f"#define SRAM_BASE {args.ram_base:#x}U\n"
            f"#define SRAM_END {args.ram_base + args.ram_size:#x}U\n"
            f"#define CORE_CLOCK_HZ {args.clock_hz}U\n"
            f"#define PAYLOAD_BYTES {len(payload)}U\n"
            f"#define PAYLOAD_FNV {digest:#x}U\n"
            f'#define ZEPHYR_MSP {application["stack"]:#x}U\n'
            f'#define ZEPHYR_RESET {application["reset"]:#x}U\n',
            encoding="utf-8",
        )
        loader = work / "loader.elf"
        subprocess.run(
            [args.cross_compile + "gcc", "-mcpu=cortex-m33", "-mthumb", "-mfloat-abi=soft",
             "-Os", "-Wall", "-Wextra", "-Werror", "-ffreestanding", "-fno-builtin",
             "-ffunction-sections", "-fdata-sections", "-fno-stack-protector", "-nostdlib",
             "-I", work, "-T", args.boot / "native.ld", "-Wl,--gc-sections,--build-id=none",
             "-o", loader, *(path for path in inputs if path.suffix != ".ld")], check=True,
        )
        raw_path = work / "loader.bin"
        subprocess.run([args.cross_compile + "objcopy", "-O", "binary", loader, raw_path],
                       check=True)
        raw = raw_path.read_bytes()
        boot = verify_loader(loader, raw, payload)
        image = raw + struct.pack("<I", FLASH_START)
        require(len(image) <= FLASH_SIZE, "ROM image exceeds the 128 KiB window")
        raw_bytes = len(image)
        image = image.ljust(FLASH_SIZE, b"\xff")
        identity, image = set_build_id(loader, image, boot["embedded_offset"])
        verify_loader(loader, image[:len(raw)], payload)
        manifest = dict(
            schema="bes2700imp-rom-v1", board="bes2700imp_devkit/bes2700imp", build=identity,
            image=args.output.name, image_sha256=sha256(image), raw_bytes=raw_bytes,
            write_base=FLASH_START, write_end_exclusive=FLASH_START + FLASH_SIZE,
            write_bytes=FLASH_SIZE, uart_baud=921600, clock_hz=args.clock_hz,
            sources=sources, zephyr=application, loader=boot,
            loader_sha256=sha256(loader.read_bytes()), payload_fnv32=digest,
        )
        binary = work / "image.bin"
        metadata = work / "image.json"
        binary.write_bytes(image)
        metadata.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        loader.replace(args.loader)
        metadata.replace(args.manifest)
        binary.replace(args.output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ("payload", "boot", "handoff", "output", "manifest", "loader"):
        parser.add_argument(f"--{option}", type=Path, required=True)
    parser.add_argument("--payload-objcopy", required=True)
    parser.add_argument("--cross-compile", required=True)
    for option in ("ram-base", "ram-size", "clock-hz"):
        parser.add_argument(f"--{option}", type=lambda value: int(value, 0), required=True)
    args = parser.parse_args()
    try:
        generate(args)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"BES2700iMP image: {error}\n")


if __name__ == "__main__":
    main()
