# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Reject malformed addresses before host packaging or target memory writes."""

import copy
import ctypes
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path

from elf import load_elf
from pack import CODE, DATA, DATA_END, pack


def fixture():
    data = bytearray(512)
    data[:7] = b"\x7fELF\x01\x01\x01"
    struct.pack_into("<HHIIIIIHHHHHH", data, 16, 2, 40, 1, CODE + 9, 52, 0, 0, 52, 32, 2, 0, 0, 0)
    struct.pack_into("<8I", data, 52, 1, 256, CODE, CODE, 16, 16, 5, 4)
    struct.pack_into("<8I", data, 84, 1, 272, DATA, CODE + 16, 4, 8, 6, 4)
    struct.pack_into("<II", data, 256, DATA_END, CODE + 9)
    data[264:272] = bytes.fromhex("00bf00bf00bf00bf")
    return data


class PackingTests(unittest.TestCase):
    def setUp(self):
        self.data = fixture()
        self.segments, self.entry = load_elf(self.data)

    def reject(self, segments=None, entry=None, data=None):
        with self.assertRaises(ValueError):
            pack(
                self.data if data is None else data,
                self.segments if segments is None else segments,
                self.entry if entry is None else entry,
            )

    def test_valid_image(self):
        payload, info = pack(self.data, self.segments, self.entry)
        self.assertEqual(info["code_bytes"], 20)
        self.assertEqual(len(payload), 84)
        self.assertEqual(payload[64:], self.data[256:276])

    def test_malformed_elf_header(self):
        for offset, fmt, value in [
            (4, "B", 2),
            (5, "B", 2),
            (18, "H", 3),
            (28, "I", 500),
            (42, "H", 16),
            (44, "H", 0),
        ]:
            with self.subTest(offset=offset):
                data = self.data.copy()
                struct.pack_into("<" + fmt, data, offset, value)
                with self.assertRaises(ValueError):
                    load_elf(data)

    def test_truncated_elf(self):
        for length in (0, 7, 51, 80):
            with self.subTest(length=length), self.assertRaises(ValueError):
                load_elf(self.data[:length])

    def test_segment_boundaries(self):
        for field, value in [
            ("vaddr", DATA_END),
            ("vaddr", CODE - 4),
            ("paddr", DATA),
            ("paddr", CODE + 4),
            ("filesz", 17),
            ("memsz", 17),
            ("offset", 510),
            ("flags", "R"),
        ]:
            with self.subTest(field=field, value=value):
                segments = copy.deepcopy(self.segments)
                segments[0][field] = value
                self.reject(segments=segments)

    def test_overlaps(self):
        for field, value in [("vaddr", CODE + 8), ("paddr", CODE + 8)]:
            with self.subTest(field=field):
                segments = copy.deepcopy(self.segments)
                segments[1][field] = value
                self.reject(segments=segments)

    def test_invalid_stack(self):
        for sp in (DATA, DATA_END + 8, DATA_END - 1):
            with self.subTest(sp=sp):
                data = self.data.copy()
                struct.pack_into("<I", data, 256, sp)
                self.reject(data=data)

    def test_invalid_entry(self):
        for reset in (CODE + 8, CODE - 1, CODE + 0x30001, CODE + 17):
            with self.subTest(reset=reset):
                data = self.data.copy()
                struct.pack_into("<I", data, 260, reset)
                self.reject(data=data, entry=reset)
        self.reject(entry=CODE + 11)

    def test_noload_data(self):
        segments = copy.deepcopy(self.segments)
        segments[1].update(filesz=0, offset=0)
        payload, _ = pack(self.data, segments, self.entry)
        self.assertEqual(len(payload), 80)
        segments[1]["vaddr"] = CODE + 16
        self.reject(segments=segments)

    def test_noload_file_offset_is_unused(self):
        data = self.data.copy()
        struct.pack_into("<8I", data, 84, 1, 0, DATA + 8, DATA + 8, 0, 24, 6, 16)
        segments, entry = load_elf(data)
        payload, _ = pack(data, segments, entry)
        self.assertEqual(len(payload), 80)
        # A file-backed segment must still satisfy alignment congruence.
        struct.pack_into("<I", data, 84 + 16, 4)
        with self.assertRaises(ValueError):
            load_elf(data)

    def test_runtime_validator(self):
        root = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "check.c"
            source.write_text(
                '#include "image_check.h"\n'
                "int check(const void *p, size_t n) { return bth_image_check(p, n); }\n"
            )
            library = Path(temp) / "check.so"
            subprocess.run(
                [
                    "cc",
                    "-shared",
                    "-fPIC",
                    "-Wall",
                    "-Werror",
                    "-I",
                    str(root),
                    str(source),
                    "-o",
                    str(library),
                ],
                check=True,
            )
            check = ctypes.CDLL(str(library)).check
            check.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
            check.restype = ctypes.c_int
            payload, _ = pack(self.data, self.segments, self.entry)
            self.assertEqual(check(payload, len(payload)), 0)
            self.assertNotEqual(check(payload, 63), 0)
            for word in range(16):
                with self.subTest(header_word=word):
                    if word == 3:  # Build identity is informational, not an authorization field.
                        continue
                    broken = bytearray(payload)
                    value = struct.unpack_from("<I", broken, word * 4)[0]
                    struct.pack_into("<I", broken, word * 4, value ^ 1)
                    self.assertNotEqual(check(bytes(broken), len(broken)), 0)
            broken = payload[:-1] + bytes([payload[-1] ^ 1])
            self.assertNotEqual(check(broken, len(broken)), 0)


if __name__ == "__main__":
    unittest.main()
