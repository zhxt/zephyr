# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Reject invalid build inputs and corrupted ROM content without a board."""

import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from build import build, linker_inputs
from inputs import dependencies, sha
from verify import validate

CROSS = os.environ.get(
    "BES_PRODUCTION_TEST_CROSS",
    "arm-none-eabi-",
)


class ProductionTests(unittest.TestCase):
    def test_linker_inputs_follow_cmake_declarations(self):
        with tempfile.TemporaryDirectory() as scratch:
            root = Path(scratch)
            source = root / "source"
            source.mkdir()
            scripts = [source / "loader.ld", source / "memory layout.ld"]
            for script in scripts:
                script.write_text("/* linker input */\n")
            (root / "link-inputs.json").write_text(json.dumps([str(p) for p in scripts]))
            with patch("build.ROOT", source):
                self.assertEqual(
                    linker_inputs(root),
                    {"local/" + p.name: p for p in scripts},
                )

    def test_invalid_or_foreign_linker_inputs_are_rejected(self):
        with tempfile.TemporaryDirectory() as scratch:
            root = Path(scratch)
            source = root / "source"
            source.mkdir()
            foreign = root / "foreign.ld"
            foreign.write_text("/* external input */\n")
            (source / "alias.ld").symlink_to(foreign)
            invalid = [
                [],
                {},
                "loader.ld",
                [None],
                ["loader.ld"],
                [str(source)],
                [str(foreign)],
                [str(source / "alias.ld")],
                [str(source / "missing.ld")],
            ]
            for paths in invalid:
                with self.subTest(paths=paths), patch("build.ROOT", source):
                    (root / "link-inputs.json").write_text(json.dumps(paths))
                    with self.assertRaises((ValueError, FileNotFoundError)):
                        linker_inputs(root)

    def test_existing_output_is_preserved_without_running_tools(self):
        with tempfile.TemporaryDirectory() as scratch:
            output = Path(scratch)
            (output / "sentinel").write_bytes(b"prior evidence")
            with (
                patch(
                    "build.subprocess.check_output",
                    side_effect=AssertionError("tool started"),
                ),
                self.assertRaisesRegex(ValueError, "already exists"),
            ):
                build(output / "missing.elf", output, output, output, "missing-")
            self.assertEqual((output / "sentinel").read_bytes(), b"prior evidence")

    def test_invalid_application_rejected_before_output_or_compiler(self):
        with tempfile.TemporaryDirectory() as scratch:
            root = Path(scratch)
            elf = root / "bad.elf"
            elf.write_bytes(b"invalid ELF")
            with (
                patch(
                    "build.subprocess.check_output",
                    side_effect=AssertionError("compiler started"),
                ),
                self.assertRaisesRegex(ValueError, "ELF"),
            ):
                build(elf, root / "output", root, root, "missing-")
            self.assertFalse((root / "output").exists())

    def test_foreign_header_is_rejected_by_real_compiler_dependency_scan(self):
        if shutil.which(CROSS + "gcc") is None:
            self.skipTest("Set BES_PRODUCTION_TEST_CROSS to the GNU Arm toolchain prefix")
        with tempfile.TemporaryDirectory() as scratch:
            root = Path(scratch)
            local = root / "local"
            local.mkdir()
            foreign = root / "foreign.h"
            foreign.write_text("#define FOREIGN 1\n")
            source = local / "source.c"
            source.write_text('#include "' + str(foreign) + '"\nint value = FOREIGN;\n')
            units = [{"name": "source", "source": str(source), "flags": []}]
            with self.assertRaisesRegex(ValueError, "Foreign compiler dependency"):
                dependencies(units, {"local": local}, CROSS)

    def test_modified_source_is_accepted_and_updates_actual_input_record(self):
        if shutil.which(CROSS + "gcc") is None:
            self.skipTest("Set BES_PRODUCTION_TEST_CROSS to the GNU Arm toolchain prefix")
        with tempfile.TemporaryDirectory() as scratch:
            root = Path(scratch)
            source = root / "source.c"
            header = root / "value.h"
            source.write_text('#include "value.h"\nint value = VALUE;\n')
            header.write_text("#define VALUE 1\n")
            units = [{"name": "source", "source": str(source), "flags": []}]
            before = dependencies(units, {"local": root}, CROSS)
            header.write_text("#define VALUE 2\n")
            after = dependencies(units, {"local": root}, CROSS)
            self.assertNotEqual(before["source"]["local/value.h"], after["source"]["local/value.h"])
            self.assertEqual(after["source"]["local/value.h"], sha(header))

    def test_external_symlink_is_rejected_by_actual_dependency_scan(self):
        if shutil.which(CROSS + "gcc") is None:
            self.skipTest("Set BES_PRODUCTION_TEST_CROSS to the GNU Arm toolchain prefix")
        with tempfile.TemporaryDirectory() as scratch:
            root = Path(scratch)
            external = root / "external.h"
            external.write_text("#define VALUE 1\n")
            local = root / "local"
            local.mkdir()
            (local / "source.h").symlink_to(external)
            source = local / "source.c"
            source.write_text('#include "source.h"\nint value = VALUE;\n')
            units = [{"name": "source", "source": str(source), "flags": []}]
            with self.assertRaisesRegex(ValueError, "Foreign compiler dependency"):
                dependencies(units, {"local": local}, CROSS)

    def test_corrupted_produced_payload_header_crc_and_trailer_are_rejected(self):
        directory = os.environ.get("BES_PRODUCTION_TEST_OUTPUT")
        if not directory:
            self.skipTest("Set BES_PRODUCTION_TEST_OUTPUT to a completed production build")
        output = Path(directory)
        image = (output / "zephyr.bin").read_bytes()
        payload = (output / "generated/bth.payload.bin").read_bytes()
        layout = json.loads((output / "generated/layout.json").read_bytes())
        required = json.loads((output / "generated/required-symbols.json").read_bytes())
        loader = output / "bootstrap/adapter"
        report = validate(loader, image, payload, layout, required)
        offset = report["payload_offset"]
        for target in ("ROM_header", "trailer", "payload_bytes", "payload_crc"):
            with self.subTest(target=target):
                changed, new_payload = bytearray(image), payload
                position = {
                    "ROM_header": 0,
                    "trailer": len(image) - 1,
                    "payload_bytes": offset + 64,
                    "payload_crc": offset + 64,
                }[target]
                changed[position] ^= 1
                if target == "payload_crc":
                    new_payload = bytes(changed[offset : offset + len(payload)])
                with self.assertRaises(ValueError):
                    validate(
                        loader,
                        bytes(changed),
                        new_payload,
                        layout,
                        required,
                    )
        with self.assertRaisesRegex(ValueError, "Missing required"):
            validate(
                loader,
                image,
                payload,
                layout,
                required + ["missing_function"],
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
