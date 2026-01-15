/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i3c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tt_smc_remoteproc, CONFIG_KERNEL_LOG_LEVEL);

#define DT_DRV_COMPAT tenstorrent_smc_remoteproc

/*
 * TODO: OCCP commands should likely be implemented as a subsystem,
 * rather than being defined here
 */

#define OCCP_APP_BASE 0x0
#define OCCP_BASE_MSG_GET_VERSION 0x0

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

BUILD_ASSERT(CONFIG_TT_SMC_REMOTEPROC_INIT_PRIO > CONFIG_I3C_CONTROLLER_INIT_PRIORITY,
	     "TT_SMC_REMOTEPROC_INIT_PRIO must be higher than I3C_CONTROLLER_INIT_PRIORITY");

struct tt_smc_remoteproc_config {
	const struct gpio_dt_spec boot_gpio; /* GPIO to signal remote ROM is ready */
};

struct tt_smc_remoteproc_data {
	struct i3c_device_desc *i3c_dev; /* I3C device descriptor for remote SMC */
};

// TODO! use Zephyr's CRC API
static inline uint8_t calculate_crc8(uint8_t *data, size_t length)
{
  // TODO: check if this is correct, initial value undefined
  uint8_t crc = 0xFF;
  const uint8_t poly = 0xD3u; /* x^8 + x^7 + x^6 + x^4 + x + 1 */
  // byte and bit order should also be specified in OCCP spec
  for (int i = 0; i < length; i++)
  {
    // calculate in little endian order
    uint8_t data_byte = data[i];
    // simputshex16("Calculating CRC8 for byte: ", data_byte);
    for (int j = 0; j < 8; j++)
    {
      uint8_t data_bit = (data_byte & 0x80u) ? 1u : 0u;
      uint8_t crc_msb = (crc & 0x80u) ? 1u : 0u;

      crc = (uint8_t)(crc << 1);
      if ((uint8_t)(data_bit ^ crc_msb))
      {
        crc ^= poly;
      }
      data_byte <<= 1;
    }
  }

  return crc;
}

int tt_smc_remoteproc_boot(const struct device *dev, uint8_t *img_data, size_t img_size)
{
	struct tt_smc_remoteproc_data *data = dev->data;
	int ret;
	struct occp_header req = {0};
	struct occp_get_version_response version_resp = {0};

	/* For now, just send a test OCCP command to ROM */
	LOG_WRN("BOOT function not yet implemented\n");

	/* Send a GET_VERSION command to test I3C */
	req.cmd_header.app_id = OCCP_APP_BASE;
	req.cmd_header.msg_id = OCCP_BASE_MSG_GET_VERSION;
	req.header_crc = calculate_crc8(((uint8_t *)&req) + 1,
					 (sizeof(req) - 1));
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
