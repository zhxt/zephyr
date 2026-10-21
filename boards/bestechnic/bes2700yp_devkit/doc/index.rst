.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

.. zephyr:board:: bes2700yp_devkit

Overview
********

This target runs a single Zephyr kernel on the BES2700YP BTH subsystem.
The BTH processor is STAR-MC1; this port uses the Cortex-M33 architecture
configuration. The board target name is retained from the existing BES2700YP
integration. It does not identify a particular PCB revision or connector layout.

The target provides a SysTick system timer and a polling UART console. It uses
the standard Zephyr reset, exception, and scheduler code. M55 firmware and
interprocessor services are not part of the application build.

Memory and boot contract
***********************

The loader must initialize the SoC before entering Zephyr. The optional source
loader packages the application into a ROM image without linking BES SDK
archives. Its hardware initialization sources retain their original notices;
this local integration does not establish their public reuse basis or complete
upstream board qualification.

The loader must:

* Set the BTH CPU and UART input clocks to 24 MHz and configure the UART0 pins.
* Provide the BTH RAM allocation below, with the code data alias at ``0x20510000``.
* Copy every file-backed ELF load segment to its physical load address, using
  the data alias for executable RAM, before entering the vector table.
* Stop the watchdogs and disable BES Ucache and the MPU, with memory barriers
  before executing the newly loaded instructions.
* Disable SysTick and peripheral interrupts, clear pending interrupts, and stop
  UART DMA. Drain the loader's UART output and transfer exclusive UART ownership
  to Zephyr.
* Enter in privileged Thread mode in the existing security domain. Set VTOR to
  ``0x00510000``, load MSP from the first vector, and branch to the Thumb reset
  vector. Clear CONTROL, BASEPRI, FAULTMASK, MSPLIM and PSPLIM. PRIMASK remains
  set until the Zephyr architecture initialization enables interrupts.

.. list-table:: Runtime allocation
   :header-rows: 1

   * - Region
     - Address
     - Size
   * - Loader reservation
     - ``0x20500000``
     - 64 KiB
   * - Application code, executable alias
     - ``0x00510000``
     - 192 KiB
   * - Application data
     - ``0x20540000``
     - 112 KiB
   * - Handoff reservation
     - ``0x2055c000``
     - 16 KiB

``CONFIG_XIP`` and ``zephyr,flash`` describe the separate code and data link
layout. The executable region is RAM populated by the loader; the board's
192 KiB code capacity is not the physical flash capacity. Zephyr copies its
initialized data and clears BSS through its normal startup path.

Building
********

Use the Zephyr SDK version supported by this Zephyr revision. No BES SDK or
binary HAL module is needed to build the Zephyr application ELF:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: bes2700yp_devkit/bes2700yp/bth
   :goals: build

The resulting ``zephyr.elf`` requires the loader contract above. The raw Zephyr
binary is not a complete ROM-bootable flash image. No west flash or debug runner
is provided yet.

ROM image
=========

To also generate a ROM image, enable :kconfig:option:`CONFIG_SOC_BES2700YP_BOOT_IMAGE`
and select the separate loader compiler. GNU Arm Embedded 10.3-2021.10 is the
currently qualified compiler, including its hard-float Cortex-M33 runtime.
The loader does not use the application's Zephyr SDK compiler. CMake, Ninja, CMSIS 6 and a Python
environment with pyelftools are required::

   west build -b bes2700yp_devkit/bes2700yp/bth samples/hello_world -- \
     -DCONFIG_SOC_BES2700YP_BOOT_IMAGE=y \
     -DBES2700YP_LOADER_CROSS_COMPILE=/path/to/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-

The default build then generates ``build/zephyr/zephyr.bes.bin`` and
``build/zephyr/bes2700yp-image.json``. The manifest identifies the application,
complete image and loader build directory under ``build/zephyr/bes2700yp/``.
The original application ELF and binary are retained. CMake declares loader
sources and compiler options. Actual source, header and tool hashes are generated
build records, without a checked-in hash lock. Source, header, CMake, toolchain
and application changes are dependencies of the image target.
Each loader attempt uses a fresh directory; successful checked artifacts may be
reused when their inputs match. Failed builds do not publish a new ROM image.

Flash ``zephyr.bes.bin`` with the external BES programmer at ``0x34000000``.
The programmer must commit the ROM image magic after writing. Enter download
mode using the board's existing procedure; after programming, start the serial
listener before exiting download mode and completely cycling power. Building
does not flash, open a serial device or control board power.

The loader retains its diagnostic records. Its startup marker uses
``zephyr_bth begin version=2 build=0x12345678 !``, with the actual image build ID.

Console and validation
**********************

UART0 uses 1152000 baud, eight data bits, no parity, one stop bit, and no flow
control. The driver supports polling input/output and error reporting. UART
interrupts, DMA, runtime format changes, clock control, and pin control are not
implemented. Confirm the physical board's serial pins and voltage from its
documentation; SoC register addresses do not specify connector wiring.

The board defaults disable FPU and MPU. Power management and other subsystems
require separate implementation and validation. Build-only test results do not
establish cold boot, clock accuracy, serial reception, or scheduler operation.
Hardware qualification must include ``samples/philosophers`` and applicable
``tests/kernel`` configurations, with the exact image and console log recorded.
