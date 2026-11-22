.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

.. zephyr:board:: bes2700imp_devkit

Overview
********

The BES2700iMP Development Kit runs Zephyr on the Arm Cortex-M33 core.
The default configuration uses a 24 MHz CPU clock, a 60 KiB SRAM window
starting at ``0x20070000``, and UART0 as the console. The board target is
``bes2700imp_devkit/bes2700imp``.

Building
********

Set up a Zephyr workspace and toolchain as described in :ref:`getting_started`.
To use an existing Arm GNU cross-compiler, see :ref:`other_x_compilers`.
A standard build produces the SRAM-linked application ELF.

.. code-block:: console

   west build -b bes2700imp_devkit -d build/bes2700imp-app samples/hello_world

To generate a ROM-bootable image, enable
:kconfig:option:`CONFIG_BES2700IMP_DEVKIT_BOOT_IMAGE` and supply the absolute
loader toolchain prefix, including its trailing hyphen. The Hello World ROM
image was tested on hardware with both the application and loader built using
`Zephyr SDK 1.0.1`_ and its ``arm-zephyr-eabi`` GNU toolchain.

.. code-block:: console

   export ZEPHYR_SDK_INSTALL_DIR=/absolute/path/to/zephyr-sdk-1.0.1
   export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
   loader_prefix="$ZEPHYR_SDK_INSTALL_DIR/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-"
   west build -b bes2700imp_devkit -d build/bes2700imp-hello samples/hello_world -- \
     -DCONFIG_BES2700IMP_DEVKIT_BOOT_IMAGE=y \
     "-DBES2700IMP_LOADER_CROSS_COMPILE=$loader_prefix"

The loader was also tested with
`GNU Arm Embedded Toolchain 10.3-2021.10 (Linux x86-64)`_. For that
toolchain, set ``loader_prefix`` to the absolute path ending in
``bin/arm-none-eabi-``.

The ROM build creates these files under the build directory's ``zephyr/``
subdirectory.

* ``zephyr.bes.bin`` is the image to program into NOR flash.
* ``zephyr.bes.elf`` is the standalone loader ELF.
* ``bes2700imp-image.json`` records the build ID, image SHA-256, and write range.

Reconfiguring the build with ROM packaging disabled removes these three
artifacts from that build directory.

Flashing and boot verification
******************************

Use an external flash tool that supports the board's BES ROM download mode
with a compatible, locally supplied RAM programmer.

#. Put the board into ROM download mode and back up the 128 KiB NOR boot
   window at ``0x28000000`` through ``0x2801ffff`` before writing it.
#. Program ``zephyr.bes.bin`` at ``0x28000000`` and read back the complete
   window. Compare the bytes or their SHA-256 with the image listed in
   ``bes2700imp-image.json``.
#. Exit download mode and power-cycle the board. Monitor UART0 at 921600
   baud, eight data bits, no parity, one stop bit, and no flow control.
#. Check the loader's ``BES2700IMP_ROM build=... stage=entry`` line against
   the manifest's ``build`` field. The sample then prints
   ``Hello World! bes2700imp_devkit/bes2700imp``.

Recovery
********

To restore the previous image, re-enter ROM download mode and write the
backup from the same board to the same boot window. Verify the readback
against the backup, exit download mode, and power-cycle the board to confirm
that the restored image boots.

.. _Zephyr SDK 1.0.1: https://github.com/zephyrproject-rtos/sdk-ng/releases/tag/v1.0.1
.. _GNU Arm Embedded Toolchain 10.3-2021.10 (Linux x86-64):
   https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
