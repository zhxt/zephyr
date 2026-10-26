# SPDX-License-Identifier: Apache-2.0
import re
import struct


def fill_build_info(data, syms):
    if not 20 <= len(data) <= 0x800000:
        raise ValueError("invalid image length")
    magic, flags, reserved, info = struct.unpack_from("<4I", data)
    base = struct.unpack_from("<I", data, len(data) - 4)[0]
    if (magic, flags, reserved, base) != (0xFFFFFFFF, 0x00040000, 0, 0x34000000):
        raise ValueError("invalid pre-download header or load-address trailer")
    if syms.get("sys_build_info") != info or not 16 < info - base < len(data) - 4:
        raise ValueError("build_info pointer outside image or differs from ELF")
    start = info - base
    end = data.find(b"\0", start, len(data) - 4)
    if end < 0:
        raise ValueError("unterminated build_info")
    text = data[start:end]
    expected = {"__userdata_start", "__aud_start", "__factory_start"}
    found = set(x.decode() for x in re.findall(rb"(\w+)=<##BASE##>", text))
    if found != expected:
        raise ValueError("unexpected address placeholders")
    for name in sorted(expected):
        value = syms.get(name)
        if value is None or not 0x30000000 <= value < 0x30800000:
            raise ValueError("missing/out-of-range sector symbol: " + name)
        old = name.encode() + b"=<##BASE##>"
        if text.count(old) != 1:
            raise ValueError("duplicate placeholder: " + name)
        text = text.replace(old, f"{name}=0x{value:08X}".encode())
    if len(text) != end - start or b"<##BASE##>" in text:
        raise ValueError("invalid build_info replacement")
    return data[:start] + text + data[end:]
