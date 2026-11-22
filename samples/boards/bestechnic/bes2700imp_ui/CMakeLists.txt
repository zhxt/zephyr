# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bes2700imp_ui)

if(NOT CONFIG_BES2700IMP_PANEL)
  message(FATAL_ERROR "BES2700iMP touch display requires the panel driver and a system clock")
endif()

target_sources(app PRIVATE src/main.c)
target_compile_options(modules__lvgl PRIVATE -Oz -flto)
