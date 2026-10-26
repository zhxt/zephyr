/*
 * Copyright (c) 2021 Bestechnic (Shanghai) Co., Ltd. All rights reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 Bestechnic (Shanghai) Co., Ltd.
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Adapted from OpenHarmony device_soc_bestechnic revision
 * 17f61a22388e6f2fbd8fe855b0ccb298b9a3bd45. See aon_revision_public.rst.
 */

#include "main_cmu_leaf_native.h"
#include "cmu_gate_native.h"

#define BOOT_AON_CHIP_ID_ADDRESS 0x40080000U
#define BOOT_AON_REVISION_MASK (0xffffU << 16)
#define BOOT_AON_REVISION_SHIFT 16U

__attribute__((section(".boot_text_flash.hal_cmu_get_aon_revision_id"), noinline))
uint32_t hal_cmu_get_aon_revision_id(void)
{
	return (boot_cmu_gate_read(BOOT_AON_CHIP_ID_ADDRESS) & BOOT_AON_REVISION_MASK) >>
		BOOT_AON_REVISION_SHIFT;
}
