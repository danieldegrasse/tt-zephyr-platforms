/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_FLASH_MSPI_NOR_H_
#define ZEPHYR_DRIVERS_FLASH_MSPI_NOR_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*spi_dw_config_t)(void);

/* Access modes for page program and read commands */
enum spi_dw_access_mode {
	SPI_DW_ACCESS_1_1_1 = 0,
	SPI_DW_ACCESS_1_1_2,
	SPI_DW_ACCESS_1_2_2,
	SPI_DW_ACCESS_2_2_2,
	SPI_DW_ACCESS_1_1_4,
	SPI_DW_ACCESS_1_4_4,
	SPI_DW_ACCESS_4_4_4,
	SPI_DW_ACCESS_1_1_8,
	SPI_DW_ACCESS_1_8_8,
	SPI_DW_ACCESS_8_8_8,
};

/* Tracks settings for interfacing with flash device */
struct spi_dw_flash {
	enum spi_dw_access_mode mode; /* Flash access mode */
	uint8_t addr_len; /* Address length in bytes */
	uint8_t read_cmd; /* Read command */
	uint8_t read_dummy; /* Dummy cycles for read command */
	uint8_t ce_cmd; /* Chip erase command */
	uint8_t se_cmd; /* Sector erase command */
	uint8_t be_cmd; /* Block erase command */
	uint8_t pp_cmd; /* Page program command */
	uint32_t ssize; /* Sector size in bytes */
	uint32_t bsize; /* Block size in bytes */
};

/* Config and data structures for flash devices */
struct spi_dw_flash_dev_config {
	const struct device *mspi_dev;
	uint32_t target_freq;
	uint8_t cs_idx;
};

struct spi_dw_flash_dev_data {
	const struct spi_dw_flash *flash_cfg;
	uint32_t flash_size; /* Flash size in bytes */
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout;
#endif
};

/* Entry for flash device settings */
struct spi_dw_flash_entry {
	uint32_t jedec_id; /* JEDEC ID of flash */
	struct spi_dw_flash flash; /* Flash settings */
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_FLASH_MSPI_NOR_H_ */
