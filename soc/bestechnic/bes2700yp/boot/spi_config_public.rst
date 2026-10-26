.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

SPI boot configuration calculation
===================================

Source and scope
------------------

The divider algorithm and common register fields in spi_config_public.c and
spi_config_public.h are adapted from the explicitly Apache-2.0 OpenHarmony
device_soc_bestechnic sources at revision
17f61a22388e6f2fbd8fe855b0ccb298b9a3bd45:

* bes2600/liteos_m/sdk/bsp/platform/hal/hal_spi.c, hal_spi_init_ctrl.
* bes2600/liteos_m/sdk/bsp/platform/hal/reg_spi.h, common SSP fields.

The original BES copyright and license notices are preserved. The upstream
Git blob IDs are 670bf0449974e3b027deae189280f280219d609f for hal_spi.c and
9ecec5bd8cb37d2a2af16c4eab35a2eab177b2b2 for reg_spi.h.

The calculation takes source clock frequency as data. It makes no hardware,
kernel or board call. Its owned input/output types are not SDK structure ABI
aliases. It uses the reference's bit-11 receive-select and four-bit chip-select
definitions without declaring a complete target SPI IP version.

Loader adapter
----------------

spi_config_adapter.c converts fields explicitly into the existing loader ABI.
The source input is 24 MHz, with the existing local selection of one or two
times that frequency. This input does not establish the remaining clock or
analog hardware contract. The adapter preserves the selected constructed CR1
word by clearing the public SOD field explicitly; it does not assume that
different register words are equivalent in master mode.

The selected startup uses 1 MHz and 6 MHz requests, equal 27-bit frames, initial
CS zero, flags 0x20, no receive framing, no DMA and zero sample delay. Their
configuration fields and controller programming sequence match the qualified
loader. The direct analog chip-select and revision/rate switching remain in
their existing implementations.

Compatibility boundary
------------------------

For flags 0..0x3f, widths 4..31, valid receive framing below 32 bits and zero
sample delay, the adapter preserves the existing calculated fields and
ordinary failure diagnostics. Slave requests retain their fatal error. Public
divider behavior is bounded against zero clock/rate and arithmetic overflow.
The existing byte-sized sample limit still reports its original diagnostic.

The following extensions are explicitly unsupported because applicable target
semantics have not been established:

* Flags 0x40/0x80, including the opaque control2 and suppress-enable behavior.
* The local zero encoding for 32-bit TX, RX or receive-frame width.
* A nonzero receive sample delay that would otherwise pass the original bound.

They enter BOOT_SPI_CONFIG_UNSUPPORTED, preserving a fatal failure instead of
reporting a successful configuration. The diagnostic's second argument
identifies flags, TX width, RX width, frame width or delay as 0, 1, 2, 3 or 4.
Known invalid ordinary inputs are still rejected by their earlier validation
where it applies before these extension checks. Remaining analog/page/PMU
source disposition and complete loader publication readiness are separate.
