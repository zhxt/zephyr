# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_compile_options(
  "$<$<COMPILE_LANGUAGE:C,CXX>:-include${CMAKE_CURRENT_SOURCE_DIR}/gcc_compat.h>"
)
set(SOC_LINKER_SCRIPT
    ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld
    CACHE INTERNAL ""
)
