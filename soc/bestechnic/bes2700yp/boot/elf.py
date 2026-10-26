# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Read ELF32 ARM program headers without external command dependencies."""

import struct


def load_elf(data):
    if len(data) < 52 or data[:7] != b"\x7fELF\x01\x01\x01":
        raise ValueError("expected little-endian ELF32")
    fields = struct.unpack_from("<HHIIIIIHHHHHH", data, 16)
    kind, machine, version, entry, phoff = fields[:5]
    ehsize, phsize, count = fields[7:10]
    if (kind, machine, version, ehsize, phsize) != (2, 40, 1, 52, 32):
        raise ValueError("expected ARM executable with standard program headers")
    if count == 0 or phoff < 52 or phoff + count * phsize > len(data):
        raise ValueError("invalid ELF program header table")
    segments = []
    for index in range(count):
        kind, offset, vaddr, paddr, filesz, memsz, flags, align = struct.unpack_from(
            "<8I", data, phoff + index * phsize
        )
        if kind != 1:
            continue
        if align not in (0, 1):
            if align & (align - 1):
                raise ValueError("invalid ELF segment alignment")
            # GNU ld may emit a NOLOAD segment with offset zero and alignment
            # inherited from its sections. No bytes are copied from that offset.
            if filesz and vaddr % align != offset % align:
                raise ValueError("invalid file-backed ELF segment alignment")
        segments.append(
            dict(
                offset=offset,
                vaddr=vaddr,
                paddr=paddr,
                filesz=filesz,
                memsz=memsz,
                flags="".join(c for bit, c in ((4, "R"), (2, "W"), (1, "E")) if flags & bit),
                align=align,
            )
        )
    return segments, entry
