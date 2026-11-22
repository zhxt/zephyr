# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

include(${ZEPHYR_BASE}/soc/bestechnic/bes2700imp/boot/integration.cmake)

if(CONFIG_I2C_DW OR CONFIG_BES2700IMP_PANEL)
  zephyr_sources(board_io.c)
endif()
