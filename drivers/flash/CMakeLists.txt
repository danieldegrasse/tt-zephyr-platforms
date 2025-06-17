# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

zephyr_library_include_directories("${ZEPHYR_BASE}/drivers/flash")

# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_FLASH_MSPI_NOR flash_mspi_nor)
# zephyr-keep-sorted-stop
