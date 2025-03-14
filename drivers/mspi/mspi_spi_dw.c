/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Based on spi_dw.c driver, which is:
 * Copyright (c) 2015 Intel Corporation.
 * Copyright (c) 2023 Synopsys, Inc. All rights reserved.
 * Copyright (c) 2023 Meta Platforms
 */

#define DT_DRV_COMPAT snps_designware_spi

#define LOG_LEVEL CONFIG_MSPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mspi_spi_dw);

#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/mspi/mspi_spi_dw.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

#include "mspi_spi_dw.h"

static void mspi_spi_dw_isr(const struct device *dev)
{
	const struct mspi_spi_dw_config *cfg = dev->config;
	struct mspi_spi_dw_data *dev_data = dev->data;
	uint32_t data;
	uint32_t risr = read_risr(dev);

	if (risr & (DW_SPI_ISR_RXOIS | DW_SPI_ISR_RXUIS)) {
		/* RX overrun or underflow */
		LOG_ERR("RX overrun or underflow");
		dev_data->err_state = risr;
		goto out;
	}

	if (risr & DW_SPI_ISR_RXFIS) {
		/* RX FIFO threshold interrupt */
		while (read_rxflr(dev)) {
			/* Pop from RX FIFO */
			data = read_dr(dev);

			/* Copy to RX buffer */
			if (dev_data->frame_size == 4) {
				*((uint32_t *)dev_data->buf_pos) = data;
			} else {
				*dev_data->buf_pos = data & 0xFF;
			}
			dev_data->buf_pos += dev_data->frame_size;
			dev_data->frame_cnt--;

			if (dev_data->frame_cnt == 0) {
				/* Wait for FIFO to drain */
				while (test_bit_sr_busy(dev)) {
				}
				/* Disable RX threshold interrupt */
				write_rxftlr(dev, 0);
				write_imr(dev, DW_SPI_IMR_MASK);
				clear_bit_ssienr(dev);
				write_ser(dev, 0);
				k_sem_give(&dev_data->isr_sem);
				goto out;
			}
		}

		if (read_rxftlr(dev) >= (dev_data->frame_cnt - 1)) {
			write_rxftlr(dev, dev_data->frame_cnt - 1);
		}
	} else if (risr & DW_SPI_ISR_TXEIS) {
		/* TX FIFO threshold interrupt */
		while (read_txflr(dev) < cfg->fifo_depth) {
			/* Push to TX FIFO */
			if (dev_data->frame_cnt == 0) {
				/* No more data to send, wait for TX FIFO to drain */
				while (test_bit_sr_busy(dev)) {
				}
				/* TX FIFO has drained, post to semaphore and disable interrupt */
				write_imr(dev, DW_SPI_IMR_MASK);
				clear_bit_ssienr(dev);
				write_ser(dev, 0);
				k_sem_give(&dev_data->isr_sem);
			}
			if (dev_data->frame_size == 4) {
				data = *((uint32_t *)dev_data->buf_pos);
			} else {
				data = *dev_data->buf_pos & 0xFF;
			}
			write_dr(dev, data);
			dev_data->buf_pos += dev_data->frame_size;
			dev_data->frame_cnt--;
		}
	}
out:
	/* Clear interrupts */
	clear_interrupts(dev);

}

/* Helper to program SPI DW for extended SPI modes */
static int mspi_spi_dw_prog_extended(const struct device *dev,
				     enum mspi_io_mode mode,
				     enum mspi_data_rate data_rate,
				     uint8_t cmd_len, uint8_t addr_len,
				     uint8_t dummy)
{
	uint32_t ctrlr0 = read_ctrlr0(dev);
	uint32_t spi_ctrlr0 = 0U;

	ctrlr0 &= ~DW_SPI_CTRLR0_FRF_RESET;
	spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_WAIT_CYCLES(dummy);
	/* 0x1 means 4 bit instruction, 0x2 means 8 bit instruction */
	spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_INST_L(cmd_len / 4);
	/* Set addr len- 0x8 means 32 bit address, 0x6 means 24 bit address */
	spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_ADDR_L(addr_len / 4);

	switch (mode) {
	case MSPI_IO_MODE_SINGLE:
		ctrlr0 |= DW_SPI_CTRLR0_FRF_STD;
		break;
	case MSPI_IO_MODE_DUAL_1_1_2:
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(0);
		ctrlr0 |= DW_SPI_CTRLR0_FRF_DUAL;
		break;
	case MSPI_IO_MODE_DUAL_1_2_2:
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(1);
		ctrlr0 |= DW_SPI_CTRLR0_FRF_DUAL;
		break;
	case MSPI_IO_MODE_DUAL:
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(2);
		ctrlr0 |= DW_SPI_CTRLR0_FRF_DUAL;
		break;
	case MSPI_IO_MODE_QUAD_1_1_4:
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(0);
		ctrlr0 |= DW_SPI_CTRLR0_FRF_QUAD;
		break;
	case MSPI_IO_MODE_QUAD_1_4_4:
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(1);
		ctrlr0 |= DW_SPI_CTRLR0_FRF_QUAD;
		break;
	case MSPI_IO_MODE_QUAD:
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(2);
		ctrlr0 |= DW_SPI_CTRLR0_FRF_QUAD;
		break;
	case MSPI_IO_MODE_OCTAL_1_1_8:
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(0);
		ctrlr0 |= DW_SPI_CTRLR0_FRF_OCTAL;
		break;
	case MSPI_IO_MODE_OCTAL_1_8_8:
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(1);
		ctrlr0 |= DW_SPI_CTRLR0_FRF_OCTAL;
		break;
	case MSPI_IO_MODE_OCTAL:
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(2);
		ctrlr0 |= DW_SPI_CTRLR0_FRF_OCTAL;
		break;
	default:
		LOG_ERR("Unsupported SPI DW access mode");
		return -EINVAL;
	}
	LOG_DBG("ctrlr0: 0x%x, spi_ctrlr0: 0x%x", ctrlr0, spi_ctrlr0);
	write_ctrlr0(dev, ctrlr0);
	if (mode != MSPI_IO_MODE_SINGLE) {
		write_spi_ctrlr0(dev, spi_ctrlr0);
	}
	return 0;
}

/* Helper to perform SPI eeprom transaction */
static int mspi_spi_dw_eeprom_transceive(const struct device *dev,
					 uint16_t opcode, uint32_t addr,
					 uint32_t addr_len, uint8_t *rx_buf,
					 uint32_t rx_len, uint8_t cs_idx,
					 enum mspi_io_mode mode,
					 uint32_t timeout)
{
	const struct mspi_spi_dw_config *cfg = dev->config;
	struct mspi_spi_dw_data *data = dev->data;
	uint32_t ctrlr0 = read_ctrlr0(dev);
	uint32_t rxftlr = (cfg->fifo_depth * 5) / 8;
	int ret;

	if (rx_len > UINT16_MAX + 1) {
		LOG_ERR("RX buffer too large");
		return -EINVAL;
	}

	if (rx_len == 0) {
		/* This function isn't needed- just call the TX function */
		return -ENOTSUP;
	}

	/* Program SPI for eeprom mode */
	ctrlr0 &= ~DW_SPI_CTRLR0_TMOD_RESET;
	ctrlr0 |= DW_SPI_CTRLR0_TMOD_EEPROM;
	if ((mode != MSPI_IO_MODE_SINGLE) && (rx_len % 4 == 0) &&
	    (cfg->max_xfer_size == 32)) {
		/*
		 * We can perform RX using 32 bit data frames, which will
		 * improve performance.
		 */
		ctrlr0 &= ~DW_SPI_CTRLR0_DFS_32_MASK;
		ctrlr0 |= DW_SPI_CTRLR0_DFS_32(32);
		/* Endian swap is needed in this mode */
		ctrlr0 |= DW_SPI_CTRLR0_SECONV;
		data->frame_size = sizeof(uint32_t);
		rx_len /= sizeof(uint32_t);
	} else {
		/* Configure SPI DFS for 8 bit frames */
		if (cfg->max_xfer_size == 32) {
			ctrlr0 &= ~DW_SPI_CTRLR0_DFS_32_MASK;
			ctrlr0 |= DW_SPI_CTRLR0_DFS_32(8);
		} else {
			ctrlr0 &= ~DW_SPI_CTRLR0_DFS_16_MASK;
			ctrlr0 |= DW_SPI_CTRLR0_DFS_16(8);
		}
		data->frame_size = sizeof(uint8_t);
	}
	write_ctrlr0(dev, ctrlr0);

	if (addr_len + 1 > cfg->fifo_depth) {
		LOG_ERR("Address length too large");
		return -EINVAL;
	}

	/* Setup RX context */
	data->buf_pos = rx_buf;
	data->frame_cnt = rx_len;
	data->err_state = 0;

	/* Program NDF */
	write_ctrlr1(dev, data->frame_cnt - 1);

	/* Program RX FIFO threshold */
	if (rxftlr > (rx_len - 1)) {
		rxftlr = rx_len - 1;
	}
	write_rxftlr(dev, rxftlr);

	/* Enable RX threshold interrupt */
	write_imr(dev, DW_SPI_IMR_RXFIM);

	/* Enable SSI and program TX FIFO */
	set_bit_ssienr(dev);
	write_dr(dev, opcode);
	if (mode == MSPI_IO_MODE_SINGLE) {
		for (int i = addr_len; i > 0; i--) {
			write_dr(dev, (addr >> ((i - 1) * 8)) & 0xFF);
		}
	} else {
		if (addr_len > 0) {
			/* For extended SPI mode, we write address as 32 bit value */
			write_dr(dev, addr);
		}
	}

	LOG_DBG("Starting eeprom transaction");

	write_ser(dev, BIT(cs_idx));
	ret = k_sem_take(&data->isr_sem, K_MSEC(timeout));
	if (ret < 0) {
		LOG_ERR("Timeout waiting for EEPROM transaction");
		return ret;
	}
	return data->err_state;
}

/* Helper to perform SPI TX transactions */
static int mspi_spi_dw_tx(const struct device *dev,
			  uint32_t opcode, uint32_t addr, uint32_t addr_len,
			  const uint8_t *tx_buf, uint32_t tx_len, uint8_t cs_idx,
			  enum mspi_io_mode mode,
			  uint32_t timeout)
{
	const struct mspi_spi_dw_config *cfg = dev->config;
	struct mspi_spi_dw_data *data = dev->data;
	uint32_t ctrlr0 = read_ctrlr0(dev);
	uint32_t txftlr = (cfg->fifo_depth * 5) / 8;
	uint32_t fifo_space = cfg->fifo_depth - (addr_len + 1);
	int ret;

	/* Program SPI for TX mode */
	ctrlr0 &= ~DW_SPI_CTRLR0_TMOD_RESET;
	ctrlr0 |= DW_SPI_CTRLR0_TMOD_TX;
	if ((mode != MSPI_IO_MODE_SINGLE) && (tx_len % 4 == 0) &&
	    (cfg->max_xfer_size == 32)) {
		/*
		 * We can perform TX using 32 bit data frames, which will
		 * improve performance.
		 */
		ctrlr0 &= ~DW_SPI_CTRLR0_DFS_32_MASK;
		ctrlr0 |= DW_SPI_CTRLR0_DFS_32(32);
		/* Endian swap is needed in this mode */
		ctrlr0 |= DW_SPI_CTRLR0_SECONV;
		data->frame_size = sizeof(uint32_t);
		tx_len /= sizeof(uint32_t);
	} else {
		/* Configure SPI DFS for 8 bit frames */
		if (cfg->max_xfer_size == 32) {
			ctrlr0 &= ~DW_SPI_CTRLR0_DFS_32_MASK;
			ctrlr0 |= DW_SPI_CTRLR0_DFS_32(8);
		} else {
			ctrlr0 &= ~DW_SPI_CTRLR0_DFS_16_MASK;
			ctrlr0 |= DW_SPI_CTRLR0_DFS_16(8);
		}
		data->frame_size = sizeof(uint8_t);
	}
	write_ctrlr0(dev, ctrlr0);

	data->buf_pos = (uint8_t *)tx_buf;
	data->frame_cnt = tx_len;
	data->err_state = 0;

	/* Setup TXE interrupt. */
	write_txftlr(dev, txftlr);

	/* Enable SSI and program TX FIFO */
	set_bit_ssienr(dev);
	write_dr(dev, opcode);
	if (mode == MSPI_IO_MODE_SINGLE) {
		for (int i = addr_len; i > 0; i--) {
			write_dr(dev, (addr >> ((i - 1) * 8)) & 0xFF);
		}
	} else {
		if (addr_len > 0) {
			/* For extended SPI mode, we write address as 32 bit value */
			write_dr(dev, addr);
		}
	}

	/* Prefill tx buffer */
	for (int i = 0; i < fifo_space; i++) {
		if (data->frame_size == 4) {
			write_dr(dev, *((uint32_t *)data->buf_pos));
		} else {
			write_dr(dev, *data->buf_pos);
		}
		data->buf_pos += data->frame_size;
		data->frame_cnt--;
	}

	/* Enable TX threshold interrupt */
	write_imr(dev, DW_SPI_IMR_TXEIM);

	LOG_DBG("Starting TX transaction");
	write_ser(dev, BIT(cs_idx));

	/* Wait for TX FIFO empty interrupt */
	ret = k_sem_take(&data->isr_sem, K_MSEC(timeout));
	if (ret < 0) {
		LOG_ERR("Timeout waiting for TX transaction");
		return ret;
	}
	return data->err_state;
}

static int mspi_spi_dw_dev_config(const struct device *dev,
			   const struct mspi_dev_id *dev_id,
			   const enum mspi_dev_cfg_mask param_mask,
			   const struct mspi_dev_cfg *cfg)
{
	struct mspi_spi_dw_data *data = dev->data;

	/*
	 * Controller does not support XIP, so we only need to save
	 * a subset of the settings
	 */

	if (dev_id->dev_idx >= CONFIG_MSPI_SPI_DW_SLAVE_COUNT) {
		return -EINVAL;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_CE_POL) &&
	    (cfg->ce_polarity != MSPI_CE_ACTIVE_LOW)) {
		return -ENOTSUP; /* Only active low supported */
	}

	if (param_mask & MSPI_DEVICE_CONFIG_FREQUENCY) {
		data->slv_cfg[dev_id->dev_idx].freq = cfg->freq;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_ENDIAN) {
		data->slv_cfg[dev_id->dev_idx].endian = cfg->endian;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_DQS) {
		data->slv_cfg[dev_id->dev_idx].dqs_enable = cfg->dqs_enable;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_IO_MODE) {
		data->slv_cfg[dev_id->dev_idx].io_mode = cfg->io_mode;
	}
	if (param_mask & MSPI_DEVICE_CONFIG_DATA_RATE) {
		data->slv_cfg[dev_id->dev_idx].data_rate = cfg->data_rate;
	}
	return 0;
}

static int mspi_spi_dw_transceive(const struct device *dev,
				  const struct mspi_dev_id *dev_id,
				  const struct mspi_xfer *req)
{
	const struct mspi_spi_dw_config *cfg = dev->config;
	struct mspi_spi_dw_data *data = dev->data;
	const struct mspi_xfer_packet *packet;
	int rc;
	uint32_t reg;

	if (dev_id->dev_idx >= CONFIG_MSPI_SPI_DW_SLAVE_COUNT) {
		return -EINVAL;
	}

	if (req->async) {
		return -ENOTSUP; /* Async not supported */
	}

	rc = k_sem_take(&data->lock, K_NO_WAIT);
	if (rc != 0) {
		return -EBUSY; /* Controller is busy */
	}

	/*
	 * Setup DQS, DDR, endian mode, and frequency as these
	 * are not per packet.
	 */

	if (data->slv_cfg[dev_id->dev_idx].freq == 0) {
		return -EINVAL; /* Frequency cannot be zero */
	}
	/* Program the frequency */
	if (data->slv_cfg[dev_id->dev_idx].freq > data->max_freq) {
		return -EINVAL; /* Frequency too high */
	}
	reg = SPI_DW_CLK_DIVIDER(cfg->clock_frequency,
				 data->slv_cfg[dev_id->dev_idx].freq);
	write_baudr(dev, reg);
	if (data->slv_cfg[dev_id->dev_idx].dqs_enable) {
		/*
		 * Set TDE to half of its max value, which means data gets
		 * driven after half of negedge cycle.
		 */
		write_txd_drive_edge(dev, reg / 4);
	}

	reg = read_spi_ctrlr0(dev);
	if (data->slv_cfg[dev_id->dev_idx].dqs_enable) {
		/* Enable DQS if required */
		reg |= DW_SPI_CTRLR0_SPI_RXDS_EN;
	} else {
		reg &= ~DW_SPI_CTRLR0_SPI_RXDS_EN;
	}

	reg &= ~(DW_SPI_CTRLR0_INST_DDR_EN | DW_SPI_CTRLR0_SPI_DDR_EN);
	switch (data->slv_cfg[dev_id->dev_idx].data_rate) {
	case MSPI_DATA_RATE_SINGLE:
		break;
	case MSPI_DATA_RATE_DUAL:
		reg |= DW_SPI_CTRLR0_INST_DDR_EN;
		__fallthrough;
	case MSPI_DATA_RATE_S_D_D:
		reg |= DW_SPI_CTRLR0_SPI_DDR_EN;
		break;
	case MSPI_DATA_RATE_S_S_D:
	default:
		return -ENOTSUP; /* Unsupported data rate */
	}
	write_spi_ctrlr0(dev, reg);

	reg = read_ctrlr0(dev);
	if (data->slv_cfg[dev_id->dev_idx].endian == MSPI_XFER_LITTLE_ENDIAN) {
		reg |= DW_SPI_CTRLR0_SECONV;
	} else {
		reg &= ~DW_SPI_CTRLR0_SECONV;
	}
	write_ctrlr0(dev, reg);

	/* Transfer each packet */
	for (uint32_t i = 0; i < req->num_packet; i++) {
		packet = &req->packets[i];

		if (packet->dir == MSPI_RX) {
			rc = mspi_spi_dw_prog_extended(dev,
				data->slv_cfg[dev_id->dev_idx].io_mode,
				data->slv_cfg[dev_id->dev_idx].data_rate,
				req->cmd_length, req->addr_length,
				req->rx_dummy);
			if (rc < 0) {
				goto out;
			}
			rc = mspi_spi_dw_eeprom_transceive(dev,
				packet->cmd, packet->address,
				req->addr_length, packet->data_buf,
				packet->num_bytes, dev_id->dev_idx,
				data->slv_cfg[dev_id->dev_idx].io_mode,
				req->timeout);
			if (rc < 0) {
				goto out;
			}
		} else {
			rc = mspi_spi_dw_prog_extended(dev,
				data->slv_cfg[dev_id->dev_idx].io_mode,
				data->slv_cfg[dev_id->dev_idx].data_rate,
				req->cmd_length, req->addr_length,
				req->tx_dummy);
			if (rc < 0) {
				goto out;
			}
			rc = mspi_spi_dw_tx(dev,
				packet->cmd, packet->address,
				req->addr_length, packet->data_buf,
				packet->num_bytes, dev_id->dev_idx,
				data->slv_cfg[dev_id->dev_idx].io_mode,
				req->timeout);
			if (rc < 0) {
				goto out;
			}
		}
	}
out:
	k_sem_give(&data->lock);
	return rc;
}

static int mspi_spi_dw_config(const struct mspi_dt_spec *spec)
{
	const struct mspi_spi_dw_config *cfg = spec->bus->config;
	struct mspi_spi_dw_data *data = spec->bus->data;

	/* Validate config settings */
	if (spec->config.channel_num != 0) {
		return -ENOTSUP;
	}
	if (spec->config.op_mode != MSPI_OP_MODE_CONTROLLER) {
		return -ENOTSUP;
	}
	if (spec->config.duplex != MSPI_HALF_DUPLEX) {
		return -ENOTSUP;
	}
	data->max_freq = MIN(spec->config.max_freq, (cfg->clock_frequency / 2));
	return 0;
}

static int mspi_spi_dw_timing_config(const struct device *dev,
				     const struct mspi_dev_id *dev_id,
				     const uint32_t param_mask,
				     void *timing_cfg)
{
	if (param_mask & TIMING_CFG_RX_DLY) {
		write_rx_sample_dly(dev, (uint32_t)timing_cfg);
		return 0;
	} else {
		return -ENOTSUP;
	}
}

static int mspi_spi_dw_init(const struct device *dev)
{
	const struct mspi_spi_dw_config *cfg = dev->config;
	struct mspi_spi_dw_data *data = dev->data;

#ifdef CONFIG_PINCTRL
	pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
#endif
	/* Mask interrupts, make sure controller is disabled */
	write_imr(dev, DW_SPI_IMR_MASK);
	clear_bit_ssienr(dev);

	/* Clear previous CTRLR0 settings */
	write_ctrlr0(dev, 0);

	k_sem_init(&data->lock, 1, 1);
	k_sem_init(&data->isr_sem, 0, 1);

	cfg->config_func();
	return 0;
}

static struct mspi_driver_api mspi_spi_dw_api = {
	.config = mspi_spi_dw_config,
	.dev_config = mspi_spi_dw_dev_config,
	.transceive = mspi_spi_dw_transceive,
	.timing_config = mspi_spi_dw_timing_config,
};

#define SPI_CFG_IRQS_SINGLE_ERR_LINE(inst)					\
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, rx_avail),		\
			    DT_INST_IRQ_BY_NAME(inst, rx_avail, priority),	\
			    mspi_spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, tx_req),		        \
			    DT_INST_IRQ_BY_NAME(inst, tx_req, priority),	\
			    mspi_spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, err_int),		\
			    DT_INST_IRQ_BY_NAME(inst, err_int, priority),	\
			    mspi_spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		irq_enable(DT_INST_IRQN_BY_NAME(inst, rx_avail));		\
		irq_enable(DT_INST_IRQN_BY_NAME(inst, tx_req));		        \
		irq_enable(DT_INST_IRQN_BY_NAME(inst, err_int));

#define SPI_CFG_IRQS_MULTIPLE_ERR_LINES(inst)					\
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, rx_avail),		\
			    DT_INST_IRQ_BY_NAME(inst, rx_avail, priority),	\
			    mspi_spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, tx_req),		        \
			    DT_INST_IRQ_BY_NAME(inst, tx_req, priority),	\
			    mspi_spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, txo_err),		\
			    DT_INST_IRQ_BY_NAME(inst, txo_err, priority),	\
			    mspi_spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, rxo_err),		\
			    DT_INST_IRQ_BY_NAME(inst, rxo_err, priority),	\
			    mspi_spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, rxu_err),		\
			    DT_INST_IRQ_BY_NAME(inst, rxu_err, priority),	\
			    mspi_spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, mst_err),		\
			    DT_INST_IRQ_BY_NAME(inst, mst_err, priority),	\
			    mspi_spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		irq_enable(DT_INST_IRQN_BY_NAME(inst, rx_avail));		\
		irq_enable(DT_INST_IRQN_BY_NAME(inst, tx_req));		        \
		irq_enable(DT_INST_IRQN_BY_NAME(inst, txo_err));		\
		irq_enable(DT_INST_IRQN_BY_NAME(inst, rxo_err));		\
		irq_enable(DT_INST_IRQN_BY_NAME(inst, rxu_err));		\
		irq_enable(DT_INST_IRQN_BY_NAME(inst, mst_err));

#define SPI_DW_IRQ_HANDLER(inst)                                   \
void spi_dw_irq_config_##inst(void)                                \
{                                                                  \
COND_CODE_1(IS_EQ(DT_NUM_IRQS(DT_DRV_INST(inst)), 1),              \
	(IRQ_CONNECT(DT_INST_IRQN(inst),                           \
		DT_INST_IRQ(inst, priority),                       \
		mspi_spi_dw_isr, DEVICE_DT_INST_GET(inst),        \
		0);                                                \
	irq_enable(DT_INST_IRQN(inst));),                          \
	(COND_CODE_1(IS_EQ(DT_NUM_IRQS(DT_DRV_INST(inst)), 3),     \
		(SPI_CFG_IRQS_SINGLE_ERR_LINE(inst)),		   \
		(SPI_CFG_IRQS_MULTIPLE_ERR_LINES(inst)))))	   \
}

#define SPI_DW_INIT(inst)                                                                   \
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(inst);))                         \
	SPI_DW_IRQ_HANDLER(inst);                                                           \
	static struct mspi_spi_dw_data spi_dw_data_##inst;                                 \
	static const struct mspi_spi_dw_config spi_dw_config_##inst = {                    \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                    \
		.clock_frequency = COND_CODE_1(                                             \
			DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, clocks), clock_frequency),   \
			(DT_INST_PROP_BY_PHANDLE(inst, clocks, clock_frequency)),           \
			(DT_INST_PROP(inst, clock_frequency))),                             \
		.config_func = spi_dw_irq_config_##inst,                                    \
		.fifo_depth = DT_INST_PROP(inst, fifo_depth),                               \
		.max_xfer_size = DT_INST_PROP(inst, max_xfer_size),                         \
		IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),)) \
	};                                                                                  \
	DEVICE_DT_INST_DEFINE(inst,                                                         \
		mspi_spi_dw_init,                                                          \
		NULL,                                                                       \
		&spi_dw_data_##inst,                                                        \
		&spi_dw_config_##inst,                                                      \
		POST_KERNEL,                                                                \
		CONFIG_MSPI_INIT_PRIORITY,                                                  \
		&mspi_spi_dw_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_DW_INIT)
