# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Prepare standalone libc inputs and record actual compiler dependencies."""

import argparse
import hashlib
import shlex
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ABI = ["-mthumb", "-mcpu=cortex-m33", "-mfpu=fpv5-sp-d16", "-mfloat-abi=hard"]
FORMAT_SOURCES = {
    "sprintf.c": "lib/libc/minimal/source/stdout/sprintf.c",
    "cbprintf_nano.c": "lib/os/cbprintf_nano.c",
    "strnlen.c": "lib/libc/common/source/string/strnlen.c",
    "string.c": "lib/libc/minimal/source/string/string.c",
}


def transform(kind, source, output, cross):
    common = ["-j", ".ARM.attributes", "-j", ".note.GNU-stack"]
    if kind == "strlen":
        subprocess.run(
            [
                cross + "objcopy",
                "-j",
                ".text.strlen",
                *common,
                "--keep-global-symbol=strlen",
                str(source),
                str(output),
            ],
            check=True,
        )
    else:
        intermediate = output.with_suffix(".section.o")
        subprocess.run(
            [
                cross + "objcopy",
                "-j",
                ".text.memmove",
                *common,
                "--keep-global-symbol=memmove",
                "--rename-section",
                ".text.memmove=.boot_text_sram.memmove",
                str(source),
                str(intermediate),
            ],
            check=True,
        )
        subprocess.run(
            [
                cross + "objcopy",
                "--add-symbol",
                "memcpy=.boot_text_sram.memmove:1,global,function",
                str(intermediate),
                str(output),
            ],
            check=True,
        )
    if subprocess.check_output([cross + "nm", "-u", str(output)], text=True).strip():
        raise ValueError("Public libc transformed object has an unresolved dependency")


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def dependencies(units, roots, cross, *, env=None):
    result = {}
    for unit in units:
        text = subprocess.check_output(
            [cross + "gcc", *unit["flags"], "-M", "-MT", "unit", unit["source"]],
            text=True,
            env=env,
        )
        paths = shlex.split(text.replace("\\\n", " ").split(":", 1)[1])
        row = {}
        for name in paths:
            path = Path(name).resolve()
            for prefix, base in roots.items():
                if path.is_relative_to(base):
                    relative = str(path.relative_to(base))
                    digest = sha(path)
                    row[prefix + "/" + relative] = digest
                    break
            else:
                raise ValueError("Foreign compiler dependency: " + name)
        result[unit["name"]] = dict(sorted(row.items()))
    return result


def prepare_sources(zephyr, generated):
    directory = generated / "format"
    directory.mkdir()
    original = {}
    for name, source in FORMAT_SOURCES.items():
        data = (zephyr / source).read_bytes()
        original[source] = hashlib.sha256(data).hexdigest()
        if name == "strnlen.c":
            before = b"while (*s != '\\0' && n < maxlen)"
            after = b"while (n < maxlen && *s != '\\0')"
            if data.count(before) != 1:
                raise ValueError(
                    "strnlen adapter context changed; review the bounded implementation"
                )
            data = data.replace(before, after)
        (directory / name).write_bytes(data)
    return original


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--transform", choices=["memory", "strlen"], required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cross-compile", required=True)
    args = parser.parse_args()
    transform(args.transform, args.input, args.output, args.cross_compile)
