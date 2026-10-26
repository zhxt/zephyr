#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Build a fixed BTH ROM image from source and explicit public inputs."""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

from elf import load_elf
from image import fill_build_info
from inputs import (
    ABI,
    ROOT,
    dependencies,
    prepare_sources,
    sha,
)
from pack import pack
from verify import loader_symbols, validate


def write(path, data):
    with path.open("x") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")


def build_info(digest):
    text = (
        "\nCHIP=best1600\nCHIP_SUBSYS=bth\nKERNEL=\nFLASH_BASE=0x34000000"
        "\nFLASH_NC_BASE=0x30000000\nFLASH_SIZE=0x800000\nNV_REC_DEV_VER=2"
        "\n__userdata_start=<##BASE##>\nUSER_SEC_SIZE=0x1000"
        "\n__aud_start=<##BASE##>\nAUD_SEC_SIZE=0"
        "\n__factory_start=<##BASE##>\nFACT_SEC_SIZE=0x1000"
        "\nBUILD_DATE=reproducible\nREV_INFO=route2-source-native:" + digest + "\n"
    )
    return (
        "/* SPDX-License-Identifier: Apache-2.0 */\n"
        'const char sys_build_info[] __attribute__((section(".build_info"))) = '
        + json.dumps(text)
        + ";\n"
    )


def run(command, output, name, env):
    write(output / (name + "-command.json"), command)
    with (output / (name + ".log")).open("x") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env, check=True)


def cmake_inputs(build):
    reply = build / ".cmake/api/v1/reply"
    index = json.loads(sorted(reply.glob("index-*.json"))[-1].read_bytes())
    data = json.loads((reply / index["reply"]["cmakeFiles-v1"]["jsonFile"]).read_bytes())
    result = {}
    for item in data["inputs"]:
        path = Path(item["path"])
        if not path.is_absolute():
            path = ROOT / path
        path = path.resolve()
        if path.is_relative_to(ROOT):
            result["local/" + str(path.relative_to(ROOT))] = path
    return result


def linker_inputs(generated):
    paths = json.loads((generated / "link-inputs.json").read_bytes())
    if not isinstance(paths, list) or not paths:
        raise ValueError("Missing loader linker inputs")
    result = {}
    for name in paths:
        if not isinstance(name, str) or not Path(name).is_absolute():
            raise ValueError("Invalid loader linker input: " + str(name))
        path = Path(name).resolve(strict=True)
        if not path.is_file() or not path.is_relative_to(ROOT):
            raise ValueError("Foreign loader linker input: " + name)
        result["local/" + str(path.relative_to(ROOT))] = path
    return result


def normalized(value, roots):
    if isinstance(value, dict):
        return {key: normalized(item, roots) for key, item in value.items()}
    if isinstance(value, list):
        return [normalized(item, roots) for item in value]
    if isinstance(value, str):
        for name, root in sorted(roots.items(), key=lambda item: -len(str(item[1]))):
            value = value.replace(str(root), "@" + name)
    return value


def build(application, output, cmsis, zephyr, cross):
    if output.exists():
        raise ValueError("Output already exists; preserve earlier artifacts")
    application = application.resolve(strict=True)
    elf_data = application.read_bytes()
    segments, entry = load_elf(elf_data)
    # Validate the application before creating a build or invoking a compiler.
    pack(elf_data, segments, entry)
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
    tools = {name: sha(Path(cross + name)) for name in ("gcc", "as", "ld", "ar", "objcopy")}
    runtime = Path(
        subprocess.check_output(
            [cross + "gcc", *ABI, "-print-file-name=libgcc.a"], text=True, env=env
        ).strip()
    )
    compiler_version = subprocess.check_output([cross + "gcc", "--version"], text=True, env=env)
    if not runtime.is_file():
        raise ValueError("GNU Arm hard-float compiler runtime is unavailable")
    output.mkdir(parents=True)
    generated, bootstrap = output / "generated", output / "bootstrap"
    generated.mkdir()
    (output / "application").mkdir()
    (output / "application/zephyr.elf").write_bytes(elf_data)
    prepared = prepare_sources(zephyr, generated)
    (generated / "build_info.c").write_text(build_info("0" * 64))
    compiler = Path(cross + "gcc").resolve().parent.parent
    roots = {
        "generated": generated,
        "local": ROOT,
        "cmsis": cmsis,
        "zephyr": zephyr,
        "compiler": compiler,
    }
    query = bootstrap / ".cmake/api/v1/query"
    query.mkdir(parents=True)
    (query / "cmakeFiles-v1").touch()
    configure = [
        "cmake",
        "-G",
        "Ninja",
        "-S",
        str(ROOT),
        "-B",
        str(bootstrap),
        "-DCROSS_COMPILE=" + cross,
        "-DGENERATED_DIR=" + str(generated),
        "-DCMSIS_ROOT=" + str(cmsis),
        "-DZEPHYR_SOURCE_ROOT=" + str(zephyr),
        "-DPython3_EXECUTABLE=" + sys.executable,
    ]
    run(configure + ["-DBES_BOOT_BUILD_ID=" + "0" * 40], output, "configure-inputs", env)
    units = json.loads((generated / "units.json").read_bytes())
    required = json.loads((generated / "required-symbols.json").read_bytes())
    initial = dependencies(units, roots, cross, env=env)
    host = [
        "build.py",
        "inputs.py",
        "verify.py",
        "elf.py",
        "pack.py",
        "image.py",
        "records.py",
    ]
    input_paths = cmake_inputs(bootstrap)
    link_paths = linker_inputs(generated)
    input_paths.update(link_paths)
    input_paths.update({"local/" + name: ROOT / name for name in host})
    for name, expected_sha in prepared.items():
        path = zephyr / name
        if sha(path) != expected_sha:
            raise ValueError("Public libc source changed during preparation")
        input_paths["zephyr/" + name] = path
    for records in initial.values():
        for name in records:
            prefix, relative = name.split("/", 1)
            if prefix != "generated":
                input_paths[name] = roots[prefix] / relative
    input_paths.update({"tool/" + name: Path(cross + name) for name in tools})
    input_paths["runtime/libgcc.a"] = runtime
    source_inputs = {name: sha(path) for name, path in input_paths.items()}
    identity = {
        "schema": 1,
        "backend": "source-cmake",
        "source_sha256": source_inputs,
        "public_dependencies": {
            u: {k: v for k, v in ds.items() if not k.startswith("generated/")}
            for u, ds in initial.items()
        },
        "units": normalized(units, roots),
        "tools_sha256": tools,
        "libgcc_sha256": sha(runtime),
        "compiler_version": compiler_version,
    }
    material = json.dumps(identity, sort_keys=True, separators=(",", ":")).encode()
    payload, layout = pack(elf_data, segments, entry, material)
    (generated / "bth.payload.bin").write_bytes(payload)
    (generated / "build_info.c").write_text(build_info(layout["build_sha256"]))
    layout.update(payload_sha256=sha(generated / "bth.payload.bin"), elf_sha256=sha(application))
    expected = dependencies(units, roots, cross, env=env)
    write(generated / "build-identity.json", identity)
    write(generated / "layout.json", layout)
    run(
        configure + ["-DBES_BOOT_BUILD_ID=" + layout["build_sha256"][:40]], output, "configure", env
    )
    if units != json.loads((generated / "units.json").read_bytes()):
        raise ValueError("CMake compilation rules changed during image preparation")
    if link_paths != linker_inputs(generated):
        raise ValueError("CMake linker inputs changed during image preparation")
    run(["cmake", "--build", str(bootstrap), "--parallel", "8"], output, "compile", env)
    from records import actual_records

    objects, actual = actual_records(bootstrap, units, roots, cross)
    if actual != expected:
        raise ValueError("Actual compiler dependency closure differs from preflight")
    write(output / "compiled-objects.json", objects)
    write(output / "compiler-dependencies.json", actual)
    raw = generated / "loader.raw.bin"
    run(
        [cross + "objcopy", "-O", "binary", str(bootstrap / "adapter"), str(raw)],
        output,
        "objcopy",
        env,
    )
    _, symbols = loader_symbols(bootstrap / "adapter")
    image = fill_build_info(raw.read_bytes(), symbols)
    report = validate(bootstrap / "adapter", image, payload, layout, required)
    if dependencies(units, roots, cross, env=env) != expected:
        raise ValueError("Production compiler inputs changed during build")
    if any(sha(input_paths[name]) != digest for name, digest in source_inputs.items()):
        raise ValueError("Build inputs changed")
    if sha(application) != layout["elf_sha256"]:
        raise ValueError("Application changed during build")
    (output / "zephyr.bin").write_bytes(image)
    report.update(
        build=layout["build"],
        image_sha256=sha(output / "zephyr.bin"),
        application_elf_sha256=sha(application),
        compiler_translation_units=len(units),
        vendor_archive_link_inputs=[],
        historical_evidence_required_by_recipe=False,
    )
    write(output / "production-audit.json", report)
    write(
        output / "build-inputs.json",
        {
            name: {"path": str(path), "sha256": source_inputs[name]}
            for name, path in input_paths.items()
        },
    )
    write(
        output / "checksums.json",
        {
            str(p.relative_to(output)): sha(p)
            for p in (
                output / "zephyr.bin",
                bootstrap / "adapter",
                bootstrap / "adapter.map",
                output / "application/zephyr.elf",
                generated / "bth.payload.bin",
                generated / "build-identity.json",
                output / "production-audit.json",
                output / "build-inputs.json",
            )
        },
    )
    print(
        "Production image built:",
        layout["build"],
        len(image),
        "bytes;",
        len(units),
        "units",
    )
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("elf", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cmsis", type=Path, required=True)
    parser.add_argument("--zephyr-source", type=Path, required=True)
    parser.add_argument("--cross-compile", required=True)
    args = parser.parse_args()
    try:
        build(
            args.elf,
            args.output.resolve(),
            args.cmsis.resolve(strict=True),
            args.zephyr_source.resolve(strict=True),
            args.cross_compile,
        )
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        parser.exit(1, str(exc) + "\n")


if __name__ == "__main__":
    main()
