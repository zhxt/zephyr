.. SPDX-License-Identifier: Apache-2.0

BES2700iMP touch display
#######################

The sample uses the ZTW623 input controller and the board's display to
show nine touch targets and a drag marker. The screen blanks after five
seconds of inactivity. Pressing the touchscreen wakes it. The next tap
can then change a target's color.

Building
********

Build for ``bes2700imp_devkit`` with a locally supplied initialization
header for panel ID ``0x560130``. Set ``BES2700IMP_PANEL_PROFILE`` to its
absolute path. The header defines the byte array ``q66cn_560130_init`` with
105 records totaling 424 bytes. Each record contains the command, delay
in milliseconds, parameter count and parameter bytes. The driver checks
the FNV-1a digest ``0x3ceca8ff`` before starting panel initialization.

The sample's devicetree overlay selects the SRAM and clock configuration
used by the display.

.. code-block:: console

   west build -b bes2700imp_devkit samples/boards/bestechnic/bes2700imp_ui -- \
     -DCONFIG_BES2700IMP_DEVKIT_BOOT_IMAGE=y \
     -DBES2700IMP_LOADER_CROSS_COMPILE=/path/to/arm-none-eabi- \
     -DBES2700IMP_PANEL_PROFILE=/path/to/panel_init.h

The ROM image is ``zephyr/zephyr.bes.bin``. It is linked to run from SRAM.
