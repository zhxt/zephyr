# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

set(BES2700YP_LOADER_CROSS_COMPILE "" CACHE STRING "GNU Arm loader toolchain prefix")
if(NOT IS_ABSOLUTE "${BES2700YP_LOADER_CROSS_COMPILE}" OR
   NOT EXISTS "${BES2700YP_LOADER_CROSS_COMPILE}gcc")
  message(FATAL_ERROR "Set BES2700YP_LOADER_CROSS_COMPILE to the checked GNU Arm compiler prefix")
endif()

# Register after Zephyr has selected its final ELF target, including builds
# which do not need a second link pass. The loader never links application code.
function(bes2700yp_add_image)
  set(loader "${ZEPHYR_BASE}/soc/bestechnic/bes2700yp/boot")
  set(script "${ZEPHYR_BASE}/scripts/build/bes2700yp_image.py")
  set(image "${PROJECT_BINARY_DIR}/zephyr.bes.bin")
  set(manifest "${PROJECT_BINARY_DIR}/bes2700yp-image.json")
  set(image_args
    --loader "${loader}"
    --zephyr-source "${ZEPHYR_BASE}"
    --cmsis "${ZEPHYR_CMSIS_6_MODULE_DIR}"
    --cross-compile "${BES2700YP_LOADER_CROSS_COMPILE}"
  )
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -B "${script}" ${image_args} --list-inputs
    RESULT_VARIABLE result
    OUTPUT_VARIABLE inputs
    ERROR_VARIABLE error
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "BES2700YP image inputs are unavailable: ${error}")
  endif()
  string(REPLACE "\n" ";" inputs "${inputs}")
  file(GLOB loader_cmake CONFIGURE_DEPENDS "${loader}/cmake/*.cmake")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${loader}/CMakeLists.txt" ${loader_cmake}
  )
  add_custom_command(
    OUTPUT "${image}" "${manifest}"
    COMMAND "${PYTHON_EXECUTABLE}" -B "${script}" ${image_args}
            --elf "$<TARGET_FILE:${logical_target_for_zephyr_elf}>"
            --work-root "${PROJECT_BINARY_DIR}/bes2700yp"
            --output "${image}"
            --manifest "${manifest}"
            --depfile "${PROJECT_BINARY_DIR}/bes2700yp-image.d"
    DEPENDS ${logical_target_for_zephyr_elf} ${inputs}
    COMMENT "Building and checking BES2700YP ROM image"
    DEPFILE "${PROJECT_BINARY_DIR}/bes2700yp-image.d"
    VERBATIM
    COMMAND_EXPAND_LISTS
  )
  add_custom_target(bes2700yp_image ALL DEPENDS "${image}" "${manifest}")
endfunction()

cmake_language(DEFER DIRECTORY "${ZEPHYR_BASE}" CALL bes2700yp_add_image)
