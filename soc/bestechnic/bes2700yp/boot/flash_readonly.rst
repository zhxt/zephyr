.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

Selected Flash read-only boot
============================

The boot path accepts JEDEC C8-60-17 and selects standard SPI read opcode 03
with three address bytes.  The 8 MiB device capacity is separate from the
BES2700YP-Y5 product's Flash capacity.  This path neither combines controllers
nor verifies the device's full address range.

The GD25LQ64C manufacturer datasheet (DS-00012, revision 3.4) documents this
JEDEC ID and the 9F, 05, 35 and 03 commands.  GD25LQ64E also uses this ID;
the ID alone does not select a unique silicon revision or ordering suffix.
Public manufacturer documents, hosted by distributors, are available at:

* https://resources.ampheo.com/static/datasheets/gigadevice/gd25lq64csigr.pdf
* https://www.novitronic.com/Datenbl%C3%A4tter/Aktive%20Komponenten/DS-00593-GD25LQ64E-Rev1.3.pdf

The existing controller and reset backend remains in use.  It exits alternate
read modes and resets volatile state; it does not issue write-enable, status
register writes, erase, or program commands on this path.  The read-only
orchestration adds no power-supply setting or quad-enable policy.  The retained
BES register backend still requires its separate source-origin review.

Command execution and status reads run from early SRAM.  Before returning to
Flash code, both the initial setup and the final status checks restore memory
mapped access.  The divider is fixed at eight, with sample delay four; those
settings do not claim a measured Flash clock rate.

The adapter checks the payload CRC before copying it, compares status registers
before and after the read, and stops on an unsupported device, read failure,
changed status, wrong divider, or CRC mismatch.  The reported ``device_bytes``
is descriptor capacity rather than a full-capacity probe.  ``full_hal_init=0``
states that the complete legacy Flash HAL was not opened.  The linked NOR support is limited
to boot timing, reset, device/status reads and transaction locking.
