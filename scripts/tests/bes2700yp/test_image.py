# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Exercise publication and cache failures without opening a serial device."""

import argparse
import importlib.util
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[2] / "build/bes2700yp_image.py"
SPEC = importlib.util.spec_from_file_location("bes2700yp_image", SCRIPT)
image = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(image)


class PublicationTests(unittest.TestCase):
    def setUp(self):
        self.scratch = tempfile.TemporaryDirectory()
        self.addCleanup(self.scratch.cleanup)
        self.root = Path(self.scratch.name)
        self.elf = self.root / "application.elf"
        self.header = self.root / "runtime.h"
        self.elf.write_bytes(b"synthetic application; not a hardware image")
        self.header.write_bytes(b"original header")
        compiler_bin = self.root / "compiler/bin"
        compiler_bin.mkdir(parents=True)
        (compiler_bin / "unused-gcc").write_bytes(b"not executed")
        self.args = argparse.Namespace(
            loader=self.root,
            zephyr_source=self.root,
            cmsis=self.root,
            cross_compile=str(compiler_bin / "unused-"),
            elf=self.elf,
            work_root=self.root / "work",
            output=self.root / "zephyr.bes.bin",
            manifest=self.root / "manifest.json",
        )
        self.input_patch = patch.object(image, "inputs", return_value={"header": self.header})
        self.input_patch.start()
        self.addCleanup(self.input_patch.stop)

    def builder(self, command, **kwargs):
        directory = Path(command[command.index("--output") + 1])
        self.assertFalse(directory.exists())
        names = (
            "zephyr.bin",
            "bootstrap/adapter",
            "bootstrap/adapter.map",
            "application/zephyr.elf",
            "generated/bth.payload.bin",
            "generated/build-identity.json",
        )
        for name in names:
            target = directory / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(name.encode() + self.elf.read_bytes())
        report = {
            "build": "0x12345678",
            "image_sha256": image.sha(directory / "zephyr.bin"),
            "application_elf_sha256": image.sha(self.elf),
        }
        (directory / "production-audit.json").write_text(json.dumps(report))
        (directory / "build-inputs.json").write_text(
            json.dumps(
                {"local/runtime.h": {"path": str(self.header), "sha256": image.sha(self.header)}}
            )
        )
        (directory / "checksums.json").write_text(
            json.dumps(
                {
                    name: image.sha(directory / name)
                    for name in (*names, "production-audit.json", "build-inputs.json")
                }
            )
        )

    def test_build_publishes_only_checked_artifacts(self):
        with patch.object(image.subprocess, "run", side_effect=self.builder) as run:
            result = image.generate(self.args)
        self.assertEqual(run.call_count, 1)
        self.assertFalse(result["hardware_validated"])
        self.assertEqual(image.sha(self.args.output), result["image_sha256"])

    def test_missing_output_is_restored_without_recompiling(self):
        with patch.object(image.subprocess, "run", side_effect=self.builder) as run:
            first = image.generate(self.args)
            self.args.output.unlink()
            self.args.manifest.unlink()
            second = image.generate(self.args)
        self.assertEqual(first, second)
        self.assertEqual(run.call_count, 1)
        self.assertEqual(image.sha(self.args.output), first["image_sha256"])

    def test_application_and_header_changes_each_start_a_new_build(self):
        with patch.object(image.subprocess, "run", side_effect=self.builder) as run:
            first = image.generate(self.args)
            self.elf.write_bytes(b"changed application")
            second = image.generate(self.args)
            self.header.write_bytes(b"changed header")
            third = image.generate(self.args)
        self.assertEqual(run.call_count, 3)
        self.assertEqual(len({r["directory"] for r in (first, second, third)}), 3)
        self.assertEqual(len({r["input_sha256"] for r in (first, second, third)}), 3)

    def test_failed_new_build_invalidates_publication_and_preserves_old_attempt(self):
        with patch.object(image.subprocess, "run", side_effect=self.builder):
            first = image.generate(self.args)
        old_image = Path(first["directory"]) / "zephyr.bin"
        old_hash = image.sha(old_image)
        self.header.write_bytes(b"unqualified change")
        with (
            patch.object(image.subprocess, "run", side_effect=subprocess.CalledProcessError(1, [])),
            self.assertRaisesRegex(ValueError, "Loader build failed"),
        ):
            image.generate(self.args)
        self.assertFalse(self.args.output.exists())
        self.assertFalse(self.args.manifest.exists())
        self.assertEqual(image.sha(old_image), old_hash)

    def test_changed_cache_object_is_rejected(self):
        with patch.object(image.subprocess, "run", side_effect=self.builder):
            first = image.generate(self.args)
        (Path(first["directory"]) / "bootstrap/adapter").write_bytes(b"corrupted")
        with (
            patch.object(image.subprocess, "run", side_effect=AssertionError("compiled")),
            self.assertRaisesRegex(ValueError, "Changed production output"),
        ):
            image.generate(self.args)

    def test_changed_inputs_during_build_cannot_be_published(self):
        def changing_builder(command, **kwargs):
            self.builder(command, **kwargs)
            self.header.write_bytes(b"modified during compile")

        with (
            patch.object(image.subprocess, "run", side_effect=changing_builder),
            self.assertRaisesRegex(ValueError, "inputs changed during build"),
        ):
            image.generate(self.args)
        self.assertFalse(self.args.output.exists())
        self.assertFalse(self.args.manifest.exists())

    def test_foreign_cache_directory_is_rejected(self):
        self.args.work_root.mkdir()
        (self.args.work_root / "current.json").write_text(
            json.dumps(
                {
                    "directory": str(self.root / "outside"),
                    "input_sha256": "unused",
                }
            )
        )
        with self.assertRaisesRegex(ValueError, "outside its build directory"):
            image.generate(self.args)

    def test_environment_cannot_add_compiler_search_paths(self):
        with patch.dict(image.os.environ, {"CPATH": "/foreign", "HAL_ROOT": "/sdk"}):
            cleaned = image.environment()
        self.assertNotIn("CPATH", cleaned)
        self.assertNotIn("HAL_ROOT", cleaned)

    def test_publication_cannot_replace_an_input(self):
        self.args.output = self.elf
        before = self.elf.read_bytes()
        with self.assertRaisesRegex(ValueError, "must not replace"):
            image.generate(self.args)
        self.assertEqual(self.elf.read_bytes(), before)
        self.assertFalse(getattr(self.args, "publication_checked", False))


if __name__ == "__main__":
    unittest.main()
