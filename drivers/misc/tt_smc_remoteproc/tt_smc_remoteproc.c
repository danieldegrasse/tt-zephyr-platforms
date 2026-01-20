/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/sys/crc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tt_smc_remoteproc, CONFIG_KERNEL_LOG_LEVEL);

#define DT_DRV_COMPAT tenstorrent_smc_remoteproc

/*
 * TODO: OCCP commands should likely be implemented as a subsystem,
 * rather than being defined here
 */

#define OCCP_APP_BASE 0x0
#define OCCP_BASE_MSG_GET_VERSION 0x0
#define OCCP_BASE_MSG_WRITE_DATA 0x2
#define OCCP_BASE_MSG_READ_DATA 0x3

#define OCCP_APP_BOOT 0x1
#define OCCP_BOOT_MSG_EXECUTE_IMAGE 0x1

struct occp_cmd_header {
	uint8_t app_id : 8;
	uint8_t msg_id : 8;
	uint8_t flags : 5;
	uint16_t length : 11;
} __packed;

struct occp_header {
	uint8_t header_crc : 8;
	bool body_crc_present : 1;
	uint32_t i3c_flags : 23;
	struct occp_cmd_header cmd_header;
} __packed;

struct occp_get_version_response {
	struct occp_header header;
	uint8_t major_version: 8;
	uint8_t minor_version: 8;
	uint32_t patch_version: 16;
	uint8_t body_crc: 8;
} __packed;

struct occp_write_data_request {
	struct occp_header header;
	uint32_t address_low;
	uint32_t address_high;
	uint32_t length: 11;
	uint32_t attributes: 5;
	uint32_t reserved: 16;
};

struct occp_read_data_request {
	struct occp_header header;
	uint32_t address_low;
	uint32_t address_high;
	uint32_t length: 11;
	uint32_t attributes: 5;
	uint32_t reserved: 16;
};

BUILD_ASSERT(CONFIG_TT_SMC_REMOTEPROC_INIT_PRIO > CONFIG_I3C_CONTROLLER_INIT_PRIORITY,
	     "TT_SMC_REMOTEPROC_INIT_PRIO must be higher than I3C_CONTROLLER_INIT_PRIORITY");

struct tt_smc_remoteproc_config {
	const struct gpio_dt_spec boot_gpio; /* GPIO to signal remote ROM is ready */
};

struct tt_smc_remoteproc_data {
	struct i3c_device_desc *i3c_dev; /* I3C device descriptor for remote SMC */
};

int tt_smc_remoteproc_boot(const struct device *dev, uint8_t *img_data, size_t img_size)
{
	struct tt_smc_remoteproc_data *data = dev->data;
	int ret;
	struct occp_header req = {0};
	struct occp_write_data_request write_req = {0};
	struct occp_read_data_request read_req = {0};
	struct occp_get_version_response version_resp = {0};

	/* For now, just send a test OCCP command to ROM */
	LOG_WRN("BOOT function not yet implemented\n");

	/* Send a GET_VERSION command to test I3C */
	req.cmd_header.app_id = OCCP_APP_BASE;
	req.cmd_header.msg_id = OCCP_BASE_MSG_GET_VERSION;
	req.header_crc = crc8((uint8_t *)&req + 1, sizeof(req) - 1, 0xD3, 0xFF, false);
	ret = i3c_write(data->i3c_dev, (uint8_t *)&req, sizeof(req));
	if (ret != 0) {
		LOG_ERR("Failed to send OCCP GET_VERSION command: %d", ret);
		return ret;
	}
	/* Read response */
	LOG_INF("Reading OCCP GET_VERSION response");
	do {
		ret = i3c_read(data->i3c_dev, (uint8_t *)&version_resp, sizeof(version_resp));
	} while (ret == -EIO);
	LOG_INF("OCCP GET_VERSION response: version %d.%d.%d",
	       version_resp.major_version,
	       version_resp.minor_version,
	       version_resp.patch_version);

	/* This image just spins at 0xc0066000 */
	unsigned char spin_bin[] = {
		0x13, 0x05, 0x00, 0x00, 0x93, 0x05, 0x10, 0x00, 0x13, 0x06, 0x20, 0x00,
		0x63, 0x00, 0x05, 0x00
	};
	unsigned int spin_bin_len = 16;

	/* Issue a WRITE_DATA command */
	write_req.header.cmd_header.app_id = OCCP_APP_BASE;
	write_req.header.cmd_header.msg_id = OCCP_BASE_MSG_WRITE_DATA;
	write_req.header.cmd_header.length = sizeof(write_req) - sizeof(write_req.header)
					       + spin_bin_len;
	write_req.header.header_crc = crc8((uint8_t *)&write_req.header + 1,
				    sizeof(write_req.header) - 1, 0xD3, 0xFF, false);
	write_req.address_low = 0xc0066000; /* Boot address for remote SMC */
	write_req.length = spin_bin_len;

	/*
	 * TODO: we should not need to copy into a buffer like this, we
	 * should be able to use the I3C transfer API. This isn't working in
	 * simulation though...
	 */
	uint8_t msg_buf[sizeof(write_req) + spin_bin_len];
	memcpy(msg_buf, &write_req, sizeof(write_req));
	memcpy(msg_buf + sizeof(write_req), spin_bin, spin_bin_len);

	ret = i3c_write(data->i3c_dev, msg_buf, sizeof(msg_buf));
	/* Without this, the I3C read following this causes issues in the OCCP processing */
	k_msleep(1000);

	do {
		ret = i3c_read(data->i3c_dev, (uint8_t *)&req, sizeof(req));
	} while (ret == -EIO);
	LOG_INF("OCCP WRITE_DATA response received");
	LOG_INF("OCCP WRITE_DATA response error code: %d", req.cmd_header.flags);

	/* Issue a READ_DATA command to verify */
	read_req.header.cmd_header.app_id = OCCP_APP_BASE;
	read_req.header.cmd_header.msg_id = OCCP_BASE_MSG_READ_DATA;
	read_req.header.cmd_header.length = sizeof(read_req) - sizeof(read_req.header);
	read_req.header.header_crc = crc8((uint8_t *)&read_req.header + 1,
				   sizeof(read_req.header) - 1, 0xD3, 0xFF, false);
	read_req.address_low = 0xc0066000; /* Boot address for remote SMC */
	read_req.length = spin_bin_len;

	ret = i3c_write(data->i3c_dev, (uint8_t *)&read_req, sizeof(read_req));
	if (ret != 0) {
		LOG_ERR("Failed to send OCCP READ_DATA command: %d", ret);
		return ret;
	}
	/* Without this, the I3C read following this causes issues in the OCCP processing */
	k_msleep(1000);

	/* Now read the response */
	uint8_t read_data[sizeof(struct occp_header) + spin_bin_len];
	do {
		ret = i3c_read(data->i3c_dev, read_data, sizeof(read_data));
	} while (ret == -EIO);
	if (ret != 0) {
		LOG_ERR("Failed to read OCCP READ_DATA response: %d", ret);
		return ret;
	}
	struct occp_header *read_resp_header = (struct occp_header *)read_data;
	LOG_INF("OCCP READ_DATA response length: %d", read_resp_header->cmd_header.length);
	if (memcmp(read_data + sizeof(struct occp_header),
		   spin_bin, spin_bin_len) != 0) {
		LOG_ERR("OCCP READ_DATA response data does not match written data");
		return -EIO;
	}

	return 0;
}

int tt_smc_remoteproc_init(const struct device *dev)
{
	const struct tt_smc_remoteproc_config *config = dev->config;
	struct tt_smc_remoteproc_data *data = dev->data;
	struct i3c_device_id remote_id = {
		.pid = data->i3c_dev->pid,
	};
	int ret;

	if (!device_is_ready(config->boot_gpio.port)) {
		return -ENODEV;
	}

	gpio_pin_configure_dt(&config->boot_gpio, GPIO_INPUT);

	/* Wait for remote GPIO to rise */
	while (gpio_pin_get_dt(&config->boot_gpio) == 0) {
		/* wait */
	}

	LOG_INF("Remote SMC ROM is ready");

	/*
	 * Remote SMC is now on the I3C bus, run dynamic adddress assignment
	 * once more so that the controller discovers it
	 */

	ret = i3c_do_daa(data->i3c_dev->bus);
	if (ret != 0) {
		LOG_INF("I3C dynamic address assignment failed: %d", ret);
		return ret;
	}

	/* Now that DAA is complete, find the device on the bus */
	data->i3c_dev = i3c_device_find(data->i3c_dev->bus, &remote_id);
	if (!data->i3c_dev) {
		LOG_INF("Remote SMC device not found on I3C bus");
		return -ENODEV;
	}

	LOG_INF("Remote SMC device found at dynamic address 0x%02x",
		data->i3c_dev->dynamic_addr);

	return 0;
}

#define TT_SMC_REMOTEPROC_DEFINE(inst)                                         \
	const struct tt_smc_remoteproc_config tt_smc_remoteproc_config_##inst = { \
		.boot_gpio = GPIO_DT_SPEC_INST_GET(inst, boot_gpios),          \
	};                                                                     \
									       \
	struct i3c_device_desc tt_smc_remoteproc_i3c_dev_##inst[] = {          \
		I3C_DEVICE_DESC_DT_INST(inst)                                  \
	};                                                                     \
									       \
	struct tt_smc_remoteproc_data tt_smc_remoteproc_data_##inst = {        \
		.i3c_dev = tt_smc_remoteproc_i3c_dev_##inst,                   \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                            \
			      tt_smc_remoteproc_init,                          \
			      NULL,                                            \
			      &tt_smc_remoteproc_data_##inst,                  \
			      &tt_smc_remoteproc_config_##inst,                \
			      POST_KERNEL,                                     \
			      CONFIG_TT_SMC_REMOTEPROC_INIT_PRIO,              \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(TT_SMC_REMOTEPROC_DEFINE)
