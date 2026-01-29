# Copyrigtht (c) 2026 Tenstorrent AI ULC
# SPDX-License-Identifier: Apache-2.0

ExternalZephyrProject_Add(
    APPLICATION bl1_secondary
    SOURCE_DIR  ${APP_DIR}/../grendel_bl1_secondary
    BUILD_ONLY 1
)

set(generated_file ${CMAKE_CURRENT_BINARY_DIR}/remote_smc_image.h)
set(source_file ${CMAKE_BINARY_DIR}/bl1_secondary/zephyr/zephyr.bin)

add_custom_command(
    OUTPUT ${generated_file}
    COMMAND
    ${PYTHON_EXECUTABLE}
    ${ZEPHYR_BASE}/scripts/build/file2hex.py
    --file ${source_file}
    > ${generated_file}
    DEPENDS bl1_secondary
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
    generate_remote_smc_image_header ALL
    DEPENDS ${generated_file}
)

set_config_bool(${DEFAULT_IMAGE} CONFIG_BOOT_REMOTE_SMC 1)

set_config_string(${DEFAULT_IMAGE} CONFIG_REMOTE_SMC_BINARY_HEADER
    "${generated_file}"
)

# Make the primary image depend on the secondary image header
add_dependencies(${DEFAULT_IMAGE} generate_remote_smc_image_header)
