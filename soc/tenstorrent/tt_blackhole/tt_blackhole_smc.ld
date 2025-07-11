/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SRAM_START CONFIG_SRAM_BASE_ADDRESS
#define SRAM_SIZE (CONFIG_SRAM_SIZE * 0x400)

#include <zephyr/arch/arc/v2/linker.ld>
