# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
set(SOC_LINKER_SCRIPT
    ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld
    CACHE INTERNAL ""
)

if(CONFIG_SOC_BES2700YP_BOOT_IMAGE)
  include("${CMAKE_CURRENT_SOURCE_DIR}/boot/integration.cmake")
endif()
