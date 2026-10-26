# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Record actual compiler and linker inputs from the current CMake build."""

import json
import re
import shlex
import subprocess
from pathlib import Path

from inputs import sha


def actual_records(build, units, roots, cross):
    commands = json.loads((build / "compile_commands.json").read_bytes())
    if len(commands) != len(units):
        raise ValueError("Undeclared compiler translation unit")
    records, expected_link = {}, {"libboot_wrappers.a"}
    for unit in units:
        prefix = "CMakeFiles/" + unit["target"] + ".dir/"
        matching = []
        for row in commands:
            args = row.get("arguments") or shlex.split(row["command"])
            name = args[args.index("-o") + 1]
            if name.startswith(prefix):
                matching.append((row, args, name))
        if len(matching) != 1:
            raise ValueError("Translation unit is not uniquely represented: " + unit["name"])
        row, args, name = matching[0]
        if Path(row["file"]).resolve() != Path(unit["source"]).resolve():
            raise ValueError("Compiler source differs from recipe")
        flags, skip = [], False
        for arg in args[1:]:
            if skip:
                skip = False
            elif arg in ("-o", "-c"):
                skip = True
            else:
                flags.append(arg)
        if sorted(flags) != sorted(unit["flags"]):
            raise ValueError("Compiler flags differ from recipe: " + unit["name"])
        path = Path(row["directory"]) / name
        records[unit["name"]] = {
            "path": str(path),
            "sha256": sha(path),
            "command": args,
        }
        if unit["kind"] in ("memory", "strlen"):
            expected_link.add(str(roots["generated"] / (unit["kind"] + ".o")))
        elif unit["kind"] != "archive":
            expected_link.add(name)
    text = subprocess.check_output(["ninja", "-C", str(build), "-t", "deps"], text=True)
    (build.parent / "ninja-dependencies.txt").write_text(text)
    actual, current = {}, None
    for line in text.splitlines():
        if line and not line.startswith(" "):
            current = next(
                (
                    u["name"]
                    for u in units
                    if line.startswith("CMakeFiles/" + u["target"] + ".dir/")
                ),
                None,
            )
            if current is None or "(VALID)" not in line:
                raise ValueError("Undeclared or stale compiler dependency")
            actual[current] = {}
        elif line.startswith("    ") and current is not None:
            path = Path(line.strip())
            if not path.is_absolute():
                path = build / path
            path = path.resolve()
            for prefix, base in roots.items():
                if path.is_relative_to(base):
                    actual[current][prefix + "/" + str(path.relative_to(base))] = sha(path)
                    break
            else:
                raise ValueError("Foreign actual compiler input: " + str(path))
    text = subprocess.check_output(
        ["ninja", "-C", str(build), "-t", "commands", "adapter"], text=True
    )
    command = shlex.split(text.splitlines()[-1])
    start = command.index(cross + "gcc")
    end = command.index("&&", start) if "&&" in command[start:] else len(command)
    args = command[start:end]
    objects = {arg for arg in args if arg.endswith((".o", ".obj", ".a"))}
    if objects != expected_link or [arg for arg in args if arg.startswith("-l")] != ["-lgcc"]:
        raise ValueError("Actual linker has missing or foreign object/library inputs")
    members = sorted(
        set(re.findall(r"\S*libgcc\.a\(([^)]+)\)", (build / "adapter.map").read_text()))
    )
    (build.parent / "compiler-runtime-members.json").write_text(
        json.dumps(members, indent=2) + "\n"
    )
    (build.parent / "link-command.json").write_text(json.dumps(args, indent=2) + "\n")
    return records, actual
