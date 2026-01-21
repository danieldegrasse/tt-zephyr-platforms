# Copyright (c) 2025 Tenstorrent AI ULC
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bl1_primary)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/remote_smc.elf
		   COMMAND ${CMAKE_C_COMPILER} -ffreestanding
		   	-nostdlib -Wl,--section-start=.text=0xc0066000
			${CMAKE_CURRENT_SOURCE_DIR}/src/remote_smc_startup.S
			-o ${CMAKE_BINARY_DIR}/remote_smc.elf
		   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/remote_smc_startup.S
)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/remote_smc.bin
		   COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_BINARY_DIR}/remote_smc.elf
			${CMAKE_BINARY_DIR}/remote_smc.bin
		   DEPENDS ${CMAKE_BINARY_DIR}/remote_smc.elf
)

add_custom_target(remote_smc_bin ALL
		  DEPENDS ${CMAKE_BINARY_DIR}/remote_smc.bin
)

set(gen_inc_dir ${ZEPHYR_BINARY_DIR}/misc/generated)
zephyr_include_directories(${gen_inc_dir})

generate_inc_file_for_target(
	app ${CMAKE_BINARY_DIR}/remote_smc.bin ${gen_inc_dir}/remote_smc_bin.inc
)

add_dependencies(app remote_smc_bin)
target_sources(app PRIVATE src/main.c)
