.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

BES2700YP standalone ROM loader
==============================

This local extraction builds the fixed BTH loader and a ROM image from an
application ELF. Its inputs are runtime sources, the standalone linker script,
explicit public Zephyr/CMSIS roots, the selected GNU Arm toolchain and the
application. The build does not import the experimental audit modules or read
SDK archives, earlier loader objects, candidates or historical evidence files.
The application ELF itself is supplied explicitly by the caller.

The source set retains the timing, formatting and vector diagnostics. Hardware
preparation, memory allocation, CPU/CRT order and the handoff are preserved.
Sources and per-target compiler options are declared in ``cmake/sources.cmake``
and ``cmake/flags.cmake``; ``cmake/linker.cmake`` owns the link options and
required interfaces. CMake generates the compilation inventory used by the
builder. No checked-in source or toolchain hash lock is required.

Build
-----

Use a Python environment with pyelftools, CMake, Ninja and the selected GNU Arm
Embedded 10.3-2021.10 toolchain. Supply the paths explicitly::

   python3 -B build.py application.elf \
     --output new-build-directory \
     --cmsis /path/to/cmsis_6 \
     --zephyr-source /path/to/zephyr \
     --cross-compile /path/to/bin/arm-none-eabi-

The output directory must not already exist. A malformed application is rejected
before invoking the compiler. The build checks actual compiler source/flags,
actual dependency records, link inputs, transformed memory/string objects,
required symbols, reserved ranges, ROM header/trailer and embedded payload CRC.
The builder records actual source, header, tool and runtime hashes as build
outputs. Normal source changes are accepted and produce a new build identity.
GNU Arm Embedded 10.3-2021.10 is the currently qualified loader compiler;
acceptance by the builder does not establish qualification of another release.

The main outputs are ``zephyr.bin``, ``bootstrap/adapter``, its map, the copied
application ELF, compiler dependency/object records and ``production-audit.json``.
The standalone identity includes CMake declarations, declared linker inputs,
compiler dependencies and builder helpers. The Zephyr image frontend tracks
its own dependency closure separately. Debug ELF paths are not part of the
ROM identity. ``build-inputs.json`` records actual input paths and hashes;
compiler runtime members are recorded separately. These generated records
do not attest to source reuse rights.

The programmer must commit the image magic when flashing. A build does not
perform flashing, reset, power control or serial capture. The board target is
``bes2700yp_devkit/bes2700yp/bth``; existing scripts may continue to use
``BES_NATIVE_BOOT_TEST_ROOT`` outside this standalone builder.

Validation boundaries
---------------------

Historical SDK-object comparisons, relocation replay, differential simulators,
qualification fixtures and hardware captures remain development validation work
outside this source package. ``production-audit.json`` checks current artifacts;
it does not attest to an external file tracer or establish cold startup.
The separate qualification report records isolation and baseline comparisons.

The original per-file notices are retained. This is a local extraction of the
source-native loader, not a completed public BES boot submission. Source reuse
basis and hardware documentation still require the review recorded by the
submission plan. The SoC build integration generates ``zephyr.bes.bin`` from
the final application ELF when ``CONFIG_SOC_BES2700YP_BOOT_IMAGE`` is enabled.

Source boundaries
-----------------

``SOURCE-ORIGIN.json`` retains the original extraction record as historical
data. Its old counts and equivalence statements do not describe the current
source set. Use each build's generated unit, dependency and input records for
the exact current inventory, including the standalone linker script.

The public adaptations and their compatibility limits are documented in
``aon_revision_public.rst``, ``spi_config_public.rst`` and
``spi_transport_public.rst``. They preserve the cited source notices. Public
CMSIS and Zephyr inputs supply architectural, memory and formatting primitives.
These scoped dependencies do not establish the rest of the target contract.

Target clock/reset/mux policy, analog framing and identity/rate selection,
FlashIP/NOR/cache control, RAM/core release and PMU/calibration still require
applicable hardware facts and a public reuse basis. Current register values,
successful boot, source-file notices and absence of vendor archives do not
establish those facts or permissions. Keep those implementation boundaries
separate when reviewing the complete boot path for external publication.

Host checks
-----------

Run ``python3 -B -m unittest discover -v``. To include the produced-ROM corruption
checks, set ``BES_PRODUCTION_TEST_OUTPUT`` to a completed build directory.
``BES_PRODUCTION_TEST_CROSS`` may select the toolchain for the foreign-header
negative check. None of these tests accesses a physical device.
