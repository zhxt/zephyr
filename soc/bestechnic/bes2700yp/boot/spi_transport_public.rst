.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

Selected SPI boot transport
============================

Source and implementation boundary
-----------------------------------

spi_transport_public.c and spi_transport_public.h adapt the programming and
polling sequences from OpenHarmony device_soc_bestechnic revision
17f61a22388e6f2fbd8fe855b0ccb298b9a3bd45. The original BES copyright and
Apache-2.0 notice are preserved. The exact sources are:

* bes2600/liteos_m/sdk/bsp/platform/hal/hal_spi.c, Git blob
  670bf0449974e3b027deae189280f280219d609f: hal_spi_set_xfer_type_id,
  the register-writing prefix of hal_spi_enable_id, hal_spi_set_cs_id,
  hal_spi_busy_id, hal_spi_send_id and hal_spi_recv_id.
* bes2600/liteos_m/sdk/bsp/platform/hal/reg_spi.h, Git blob
  9ecec5bd8cb37d2a2af16c4eab35a2eab177b2b2: common register offsets,
  enable/busy/FIFO predicates, DSS and the four-bit chip-select field.

The source supplies a common algorithm, not a complete BES2700YP SPI IP
declaration. The core receives a base address and calls the existing MMIO
hooks. It uses Zephyr little-endian helpers for unaligned ordinary RAM instead
of importing the public byte-copy helpers. It calls no kernel, CMU, IRQ or
analog routine and introduces no external runtime library.

Selected operation
--------------------

The loader adapter selects controller zero, equal 27-bit master frames,
TX setup, zero control2/sample delay and no DMA. Its current clock selector is
one for both 1 MHz and 6 MHz requests. Send and receive consume exactly one
four-byte non-NULL RAM frame. The analog caller supplies that frame while its
existing interrupt lock serializes bus ownership.

The core preserves CR0, RX control, CR1, prescaler and DMA programming order.
It polls enable, TX space, RX availability and busy exactly as the selected
qualified path. Receive drains stale FIFO data, writes UINT32_MAX to interrupt
clear and then exchanges the frame. The original eight-byte receive window
cannot constrain this four-byte transaction, so the core needs no long-transfer
window rule. It does not implement NULL-command filler behavior.

The public implementation reads live CR0 and DMA before polling and reads the
slave-mode bit around its slave-output helpers. The adapter instead uses its
owned cached configuration and requires the master/no-DMA setup established
before handoff. Slave-output changes are excluded. The public-reference host
comparison records and verifies those additional reads separately; it does
not claim the entire unadapted public API has an identical MMIO trace.

spi_control_native.c retains the loader state ABI and selected clock/reset
adapter; spi_poll_native.c retains the entry points and cached-DMA diagnostics.
The fixed controller base, module selectors 35/24 and CMU implementation retain
their separate hardware/source review. The original three-element state arrays
remain the same size. External-controller mappings and their clock branches
are not selected by this adapter.

Failure and compatibility scope
--------------------------------

Zero or misaligned length retains return -1. Receive on an initially busy
controller retains -11, and disabling the controller before completion retains
return one. Existing cached-DMA fatal reasons 13/14 remain in their original
order. The CS upper bound retains fatal reason 12. Hardware polling remains
unbounded, as in the qualified and public implementations; no timeout unit or
physical progress guarantee is inferred from a successful boot.

Unsupported controller, receive setup, frame, aligned long length, NULL buffer,
mode and clock selection use fatal reasons 15 through 21 respectively. The
control adapter rejects unequal/non-27-bit words, nonzero control2/sample delay,
DMA, slave/loopback and a clock selector other than one. It never writes the
unverified receive-sampling registers at offsets 0x28/0x2c.

The analog frame, page-select/restore, chip identity and revision-based rate
switching remain unchanged and still require applicable public source/facts.
This selected common transport does not complete their publication basis or
extend support to slave, DMA, IRQ, other controllers or long transfers.
