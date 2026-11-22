# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

function(bes2700imp_add_image)
  set(output "${PROJECT_BINARY_DIR}/zephyr.bes.bin")
  set(manifest "${PROJECT_BINARY_DIR}/bes2700imp-image.json")
  set(loader "${PROJECT_BINARY_DIR}/zephyr.bes.elf")

  if(NOT CONFIG_BES2700IMP_DEVKIT_BOOT_IMAGE)
    # Drop the previous ROM image when reconfiguring a non-ROM build.
    file(REMOVE "${output}" "${manifest}" "${loader}")
    return()
  endif()

  set(BES2700IMP_LOADER_CROSS_COMPILE "" CACHE STRING "GNU Arm loader toolchain prefix")
  if(NOT IS_ABSOLUTE "${BES2700IMP_LOADER_CROSS_COMPILE}" OR
     NOT EXISTS "${BES2700IMP_LOADER_CROSS_COMPILE}gcc" OR
     NOT EXISTS "${BES2700IMP_LOADER_CROSS_COMPILE}objcopy")
    message(FATAL_ERROR "Set BES2700IMP_LOADER_CROSS_COMPILE to the GNU Arm toolchain prefix")
  endif()

  set(boot "${ZEPHYR_BASE}/soc/bestechnic/bes2700imp/boot")
  set(script "${ZEPHYR_BASE}/scripts/build/bes2700imp_image.py")
  set(handoff "${boot}/handoff.S")
  dt_chosen(sram PROPERTY "zephyr,sram")
  if(NOT DEFINED sram)
    message(FATAL_ERROR "BES2700iMP ROM image requires zephyr,sram")
  endif()
  dt_reg_addr(ram_base PATH "${sram}")
  dt_reg_size(ram_size PATH "${sram}")
  dt_prop(clock_hz PATH "/cpus/cpu@0" PROPERTY "clock-frequency" REQUIRED)

  add_custom_command(
    OUTPUT "${output}" "${manifest}" "${loader}"
    COMMAND "${PYTHON_EXECUTABLE}" -B "${script}"
            --payload "$<TARGET_FILE:${logical_target_for_zephyr_elf}>"
            --payload-objcopy "${CMAKE_OBJCOPY}"
            --cross-compile "${BES2700IMP_LOADER_CROSS_COMPILE}"
            --ram-base "${ram_base}" --ram-size "${ram_size}"
            --clock-hz "${clock_hz}"
            --boot "${boot}" --handoff "${handoff}"
            --output "${output}" --manifest "${manifest}" --loader "${loader}"
    DEPENDS ${logical_target_for_zephyr_elf} "${script}" "${handoff}"
            "${boot}/native.c" "${boot}/native_entry.S" "${boot}/payload.S" "${boot}/native.ld"
            "${BES2700IMP_LOADER_CROSS_COMPILE}gcc" "${BES2700IMP_LOADER_CROSS_COMPILE}objcopy"
    COMMENT "Building and checking BES2700iMP ROM image"
    VERBATIM
  )
  add_custom_target(bes2700imp_image ALL DEPENDS "${output}" "${manifest}" "${loader}")
endfunction()

cmake_language(DEFER DIRECTORY "${ZEPHYR_BASE}" CALL bes2700imp_add_image)
