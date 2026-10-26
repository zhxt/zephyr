#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Generate a checked BES2700YP ROM image from the final application ELF."""

import argparse
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ABI = ["-mthumb", "-mcpu=cortex-m33", "-mfpu=fpv5-sp-d16", "-mfloat-abi=hard"]
HELPERS = (
    "build.py",
    "inputs.py",
    "records.py",
    "verify.py",
    "elf.py",
    "pack.py",
    "image.py",
    "CMakeLists.txt",
)


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def environment():
    env = dict(os.environ)
    for key in (
        "HAL_ROOT",
        "CFLAGS",
        "CPPFLAGS",
        "LDFLAGS",
        "CPATH",
        "C_INCLUDE_PATH",
        "CPLUS_INCLUDE_PATH",
        "LIBRARY_PATH",
        "GCC_EXEC_PREFIX",
        "COMPILER_PATH",
    ):
        env.pop(key, None)
    return env


def roots(loader, zephyr, cmsis, cross):
    compiler = Path(cross + "gcc").resolve(strict=True).parent.parent
    return {"loader": loader, "zephyr": zephyr, "cmsis": cmsis, "compiler": compiler}


def inputs(loader, zephyr, cmsis, cross):
    """Monitor loader files; CMake alone selects compiled translation units."""
    paths = {"frontend": Path(__file__).resolve()}
    for path in loader.rglob("*"):
        if path.is_file() and (
            path.suffix in (".c", ".h", ".S", ".ld", ".cmake")
            or str(path.relative_to(loader)) in HELPERS
        ):
            if path.is_symlink() or not path.resolve().is_relative_to(loader):
                raise ValueError("Foreign loader input: " + str(path))
            paths["loader/" + str(path.relative_to(loader))] = path
    spec = importlib.util.spec_from_file_location("bes_loader_inputs", loader / "inputs.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for name in module.FORMAT_SOURCES.values():
        paths["zephyr/" + name] = zephyr / name
    for name in ("gcc", "as", "ld", "ar", "objcopy"):
        paths["tool/" + name] = Path(cross + name)
    runtime = subprocess.check_output(
        [cross + "gcc", *ABI, "-print-file-name=libgcc.a"],
        text=True,
        env=environment(),
    ).strip()
    paths["runtime/libgcc.a"] = Path(runtime).resolve(strict=True)
    for path in paths.values():
        if not path.is_file() or any(c in str(path) for c in "\n\r;$"):
            raise ValueError("Missing or unsupported image input: " + str(path))
    return dict(sorted(paths.items()))


def write_depfile(args, paths):
    depfile = getattr(args, "depfile", None)
    if depfile is None:
        return

    def escape(path):
        return str(path).replace("$", "$$").replace("#", "\\#").replace(" ", "\\ ")

    targets = escape(args.output) + " " + escape(args.manifest)
    body = targets + ": " + " ".join(escape(path) for path in paths.values()) + "\n"
    atomic_write(depfile, body.encode())


def add_recorded_inputs(paths, records, allowed):
    for name, record in records.items():
        path = Path(record["path"]).resolve()
        if not any(path.is_relative_to(root) for root in allowed.values()) and (
            paths.get(name, Path("/missing")).resolve() != path
        ):
            raise ValueError("Foreign recorded image dependency: " + str(path))
        if path.is_file():
            paths[name.replace("local/", "loader/", 1) if name.startswith("local/") else name] = (
                path
            )


def fingerprint(paths):
    return {name: sha(path) for name, path in paths.items()}


def digest(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def atomic_write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            temporary.unlink(missing_ok=True)
            raise
    try:
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def checked_image(directory):
    checksums = json.loads((directory / "checksums.json").read_bytes())
    required = {
        "zephyr.bin",
        "bootstrap/adapter",
        "bootstrap/adapter.map",
        "application/zephyr.elf",
        "generated/bth.payload.bin",
        "generated/build-identity.json",
        "production-audit.json",
        "build-inputs.json",
    }
    if set(checksums) != required:
        raise ValueError("Unexpected production checksum inventory")
    for name, expected in checksums.items():
        if sha(directory / name) != expected:
            raise ValueError("Changed production output: " + name)
    report = json.loads((directory / "production-audit.json").read_bytes())
    if report["image_sha256"] != checksums["zephyr.bin"]:
        raise ValueError("Image does not match production audit")
    return report


def generate(args):
    paths = inputs(args.loader, args.zephyr_source, args.cmsis, args.cross_compile)
    paths["application"] = args.elf.resolve(strict=True)
    destinations = [args.output, args.manifest]
    if getattr(args, "depfile", None) is not None:
        destinations.append(args.depfile)
    output_paths = {path.resolve() for path in destinations}
    if len(output_paths) != len(destinations) or output_paths.intersection(
        p.resolve() for p in paths.values()
    ):
        raise ValueError("Image outputs must not replace each other or a build input")
    args.publication_checked = True
    args.work_root.mkdir(parents=True, exist_ok=True)
    cache = args.work_root / "current.json"
    previous = None
    if cache.is_file():
        previous = json.loads(cache.read_bytes())
        directory = Path(previous["directory"])
        if not directory.resolve().is_relative_to(args.work_root):
            raise ValueError("Image cache is outside its build directory")
        records = {name: {"path": path} for name, path in previous.get("input_paths", {}).items()}
        add_recorded_inputs(
            paths, records, roots(args.loader, args.zephyr_source, args.cmsis, args.cross_compile)
        )
    before = fingerprint(paths)
    key = digest(before)
    if previous is not None and previous["input_sha256"] == key:
        report = checked_image(directory)
        if fingerprint(paths) != before:
            raise ValueError("Image inputs changed during cache verification")
        write_depfile(args, paths)
        atomic_write(args.output, (directory / "zephyr.bin").read_bytes())
        atomic_write(args.manifest, json.dumps(previous, indent=2).encode() + b"\n")
        print("BES2700YP image reused:", report["build"])
        return previous

    # Every invocation of the production builder gets a new, absent output
    # directory. Failed attempts remain available for diagnosis.
    attempt = Path(tempfile.mkdtemp(prefix=key[:12] + "-", dir=args.work_root))
    directory = attempt / "image"
    args.output.unlink(missing_ok=True)
    args.manifest.unlink(missing_ok=True)
    command = [
        sys.executable,
        "-B",
        str(args.loader / "build.py"),
        str(args.elf),
        "--output",
        str(directory),
        "--cmsis",
        str(args.cmsis),
        "--zephyr-source",
        str(args.zephyr_source),
        "--cross-compile",
        args.cross_compile,
    ]
    with (attempt / "frontend.log").open("xb") as log:
        try:
            subprocess.run(
                command, stdout=log, stderr=subprocess.STDOUT, env=environment(), check=True
            )
        except subprocess.CalledProcessError as exc:
            raise ValueError("Loader build failed; see " + str(attempt / "frontend.log")) from exc
    report = checked_image(directory)
    if fingerprint(paths) != before:
        raise ValueError("Image inputs changed during build")
    actual = json.loads((directory / "build-inputs.json").read_bytes())
    final_paths = inputs(args.loader, args.zephyr_source, args.cmsis, args.cross_compile)
    final_paths["application"] = args.elf.resolve(strict=True)
    allowed = roots(args.loader, args.zephyr_source, args.cmsis, args.cross_compile)
    add_recorded_inputs(final_paths, actual, allowed)
    for record in actual.values():
        if sha(record["path"]) != record["sha256"]:
            raise ValueError("Recorded compiler input changed during publication")
    final = fingerprint(final_paths)
    manifest = {
        "schema": 2,
        "input_sha256": digest(final),
        "inputs": final,
        "input_paths": {name: str(path) for name, path in final_paths.items()},
        "directory": str(directory),
        "build": report["build"],
        "image_sha256": report["image_sha256"],
        "application_elf_sha256": report["application_elf_sha256"],
        "hardware_validated": False,
    }
    encoded = json.dumps(manifest, indent=2).encode() + b"\n"
    write_depfile(args, final_paths)
    atomic_write(cache, encoded)
    atomic_write(args.output, (directory / "zephyr.bin").read_bytes())
    atomic_write(args.manifest, encoded)
    print("BES2700YP ROM image:", args.output, report["build"])
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--loader", type=Path, required=True)
    parser.add_argument("--zephyr-source", type=Path, required=True)
    parser.add_argument("--cmsis", type=Path, required=True)
    parser.add_argument("--cross-compile", required=True)
    parser.add_argument("--list-inputs", action="store_true")
    parser.add_argument("--elf", type=Path)
    parser.add_argument("--work-root", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--depfile", type=Path)
    args = parser.parse_args()
    if not args.list_inputs and any(
        getattr(args, name) is None for name in ("elf", "work_root", "output", "manifest")
    ):
        parser.error("--elf, --work-root, --output and --manifest are required")
    try:
        for name in ("loader", "zephyr_source", "cmsis"):
            setattr(args, name, getattr(args, name).resolve(strict=True))
        if args.list_inputs:
            print(
                "\n".join(
                    str(p)
                    for p in inputs(
                        args.loader, args.zephyr_source, args.cmsis, args.cross_compile
                    ).values()
                )
            )
        else:
            for name in ("elf", "work_root", "output", "manifest"):
                setattr(args, name, getattr(args, name).resolve())
            generate(args)
    except (ValueError, OSError, KeyError, subprocess.CalledProcessError) as exc:
        if getattr(args, "publication_checked", False):
            args.output.unlink(missing_ok=True)
            args.manifest.unlink(missing_ok=True)
        parser.exit(1, str(exc) + "\n")


if __name__ == "__main__":
    main()
