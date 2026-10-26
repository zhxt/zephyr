.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

AON revision field
===================

aon_revision_public.c adapts the revision getter and its field/address definitions
from OpenHarmony device_soc_bestechnic revision
17f61a22388e6f2fbd8fe855b0ccb298b9a3bd45. The original BES Apache-2.0 notice is
preserved. The source paths below are under bes2600/liteos_m/sdk/bsp/platform/hal:

* best2003/hal_cmu_best2003.c, hal_cmu_get_aon_revision_id: Git blob
  108879c6ea7fb7984e760764b39f21e464c07ca3.
* best2003/reg_aoncmu_best2003.h, CHIP_ID at offset zero and
  AON_CMU_REVISION_ID mask 0xffff0000, shift 16: Git blob
  e4e687a360cda9dd1b1d6148143cf0b727f0df71.
* best2003/plat_addr_map_best2003.h, AON_CMU_BASE 0x40080000: Git blob
  6dd3a0cc17edee86a5f45474c6e7caf3b3272ca2.

The getter performs one 32-bit read through the existing MMIO hook and returns
the upper 16 bits without modifying interrupt state. This matches the selected
loader's field, register and return ABI. The unsigned mask avoids signed-shift
arithmetic in the imported field expression.

This scoped reuse does not establish applicability of the complete BEST2003
AON/CMU layout to BES2700YP. It does not define the analog revision threshold,
CS eight, rate-switch policy or PMU metal revision. Those target contracts and
their source review remain separate. No clock, reset, voltage or calibration
write is introduced by this getter.
