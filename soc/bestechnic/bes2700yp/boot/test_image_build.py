# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Exercise image reuse, discovered headers and failed publication without hardware."""

import argparse
import importlib.util
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[4] / "scripts/build/bes2700yp_image.py"
SPEC = importlib.util.spec_from_file_location("bes_image_builder", SCRIPT)
image = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(image)


class ImageBuildTests(unittest.TestCase):
    def setUp(self):
        self.scratch = tempfile.TemporaryDirectory()
        self.addCleanup(self.scratch.cleanup)
        root = Path(self.scratch.name)
        self.local = root / "loader"
        self.public = root / "public"
        self.local.mkdir()
        self.public.mkdir()
        self.source = self.local / "source.c"
        self.header = self.public / "value.h"
        self.source.write_text("source v1")
        self.header.write_text("header v1")
        self.application = root / "app.elf"
        self.application.write_bytes(b"app")
        self.args = argparse.Namespace(
            loader=self.local,
            zephyr_source=self.public,
            cmsis=self.public,
            cross_compile="unused-",
            elf=self.application,
            work_root=root / "attempts",
            output=root / "image.bin",
            manifest=root / "image.json",
            depfile=root / "image.d",
        )
        self.calls = 0
        self.addCleanup(patch.stopall)
        patch.object(
            image, "inputs", side_effect=lambda *args: {"loader/source.c": self.source}
        ).start()
        patch.object(
            image, "roots", return_value={"loader": self.local, "zephyr": self.public}
        ).start()
        patch.object(image.subprocess, "run", side_effect=self.builder).start()

    def builder(self, command, **_kwargs):
        self.calls += 1
        directory = Path(command[command.index("--output") + 1])
        directory.mkdir()
        records = {"local/source.c": {"path": str(self.source), "sha256": image.sha(self.source)}}
        if self.header.exists():
            records["zephyr/value.h"] = {"path": str(self.header), "sha256": image.sha(self.header)}
        content = self.source.read_bytes() + (
            self.header.read_bytes() if self.header.exists() else b""
        )
        files = {
            "zephyr.bin": content,
            "bootstrap/adapter": b"loader ELF",
            "bootstrap/adapter.map": b"link map",
            "application/zephyr.elf": self.application.read_bytes(),
            "generated/bth.payload.bin": b"payload",
            "generated/build-identity.json": b"{}",
            "build-inputs.json": json.dumps(records).encode(),
        }
        for name, data in files.items():
            path = directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        report = {
            "build": "0x" + image.sha(directory / "zephyr.bin")[:8],
            "image_sha256": image.sha(directory / "zephyr.bin"),
            "application_elf_sha256": image.sha(self.application),
        }
        (directory / "production-audit.json").write_text(json.dumps(report))
        files["production-audit.json"] = b""
        checksums = {name: image.sha(directory / name) for name in files}
        (directory / "checksums.json").write_text(json.dumps(checksums))

    def test_unchanged_build_is_reused_with_discovered_header(self):
        first = image.generate(self.args)
        second = image.generate(self.args)
        self.assertEqual(self.calls, 1)
        self.assertEqual(first, second)
        self.assertIn(str(self.header), self.args.depfile.read_text())

    def test_discovered_public_header_edit_rebuilds_and_changes_identity(self):
        first = image.generate(self.args)
        self.header.write_text("header v2")
        second = image.generate(self.args)
        self.assertEqual(self.calls, 2)
        self.assertNotEqual(first["build"], second["build"])
        self.assertNotEqual(first["input_sha256"], second["input_sha256"])

    def test_local_source_edit_rebuilds_without_updating_a_lock(self):
        first = image.generate(self.args)
        self.source.write_text("source v2")
        second = image.generate(self.args)
        self.assertEqual(self.calls, 2)
        self.assertNotEqual(first["build"], second["build"])

    def test_application_edit_invalidates_cached_image(self):
        first = image.generate(self.args)
        self.application.write_bytes(b"changed application")
        second = image.generate(self.args)
        self.assertEqual(self.calls, 2)
        self.assertNotEqual(first["application_elf_sha256"], second["application_elf_sha256"])

    def test_removed_obsolete_header_does_not_prevent_new_build(self):
        image.generate(self.args)
        self.header.unlink()
        self.source.write_text("source without old include")
        current = image.generate(self.args)
        self.assertEqual(self.calls, 2)
        self.assertNotIn("zephyr/value.h", current["inputs"])

    def test_failed_rebuild_removes_previous_published_image(self):
        image.generate(self.args)
        self.source.write_text("invalid source")
        with (
            patch.object(
                image.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "cc")
            ),
            self.assertRaisesRegex(ValueError, "Loader build failed"),
        ):
            image.generate(self.args)
        self.assertFalse(self.args.output.exists())
        self.assertFalse(self.args.manifest.exists())

    def test_changed_cached_artifact_cannot_be_published_as_valid(self):
        first = image.generate(self.args)
        (Path(first["directory"]) / "zephyr.bin").write_bytes(b"corrupted")
        with self.assertRaisesRegex(ValueError, "Changed production output"):
            image.generate(self.args)

    def test_foreign_cached_dependency_is_rejected(self):
        first = image.generate(self.args)
        foreign = self.application.parent / "foreign.h"
        foreign.write_text("foreign")
        cache = self.args.work_root / "current.json"
        first["input_paths"]["zephyr/foreign.h"] = str(foreign)
        cache.write_text(json.dumps(first))
        with self.assertRaisesRegex(ValueError, "Foreign recorded"):
            image.generate(self.args)

    def test_depfile_escapes_spaces_hashes_and_dollars(self):
        name = self.local / "a b#$.h"
        name.write_text("header")
        image.write_depfile(self.args, {"header": name})
        self.assertIn("a\\ b\\#$$.h", self.args.depfile.read_text())

    def test_depfile_cannot_overwrite_application(self):
        original = self.application.read_bytes()
        self.args.depfile = self.application
        with self.assertRaisesRegex(ValueError, "must not replace"):
            image.generate(self.args)
        self.assertEqual(self.calls, 0)
        self.assertEqual(self.application.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
