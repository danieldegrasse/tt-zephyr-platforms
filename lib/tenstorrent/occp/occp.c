/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Open Chiplet Configuration Protocol (OCCP) implementation
 *
 * OCCP is a protocol used to communicate with chiplets during early boot.
 * It can support multiple transport layers, implemented as backends within
 * this subsystem.
 */

#include <tenstorrent/occp.h>
#include "occp_private.h"

#include <zephyr/sys/crc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(occp, CONFIG_OCCP_LOG_LEVEL);

uint8_t occp_rw_buffer[OCCP_MAX_RW_BUFFER_SIZE];

/**
 * @brief Get OCCP protocol version
 * @param backend OCCP backend to use
 * @param major Pointer to store major version
 * @param minor Pointer to store minor version
 * @param patch Pointer to store patch version
 * @return 0 on success, negative error code on failure
 */
int occp_get_version(const struct occp_backend *backend, uint8_t *major, uint8_t *minor,
		     uint8_t *patch)
{
	struct occp_header req = {0};
	struct occp_get_version_response version_resp = {0};
	int ret;

	/* Send a GET_VERSION command */
	req.cmd_header.app_id = OCCP_APP_BASE;
	req.cmd_header.msg_id = OCCP_BASE_MSG_GET_VERSION;
	req.header_crc = crc8((uint8_t *)&req + 1, sizeof(req) - 1, 0xD3, 0xFF, false);
	ret = backend->send(backend, (uint8_t *)&req, sizeof(req));
	if (ret != 0) {
		LOG_ERR("Failed to send OCCP GET_VERSION command: %d", ret);
		return ret;
	}
	/* Read response */
	LOG_DBG("Reading OCCP GET_VERSION response");
	ret = backend->receive(backend, (uint8_t *)&version_resp, sizeof(version_resp));
	if (ret != 0) {
		LOG_ERR("Failed to read OCCP GET_VERSION response: %d", ret);
		return ret;
	}
	*major = version_resp.major_version;
	*minor = version_resp.minor_version;
	*patch = version_resp.patch_version;
	return 0;
}

/**
 * @brief Write data to OCCP device
 * @param backend OCCP backend to use
 * @param address Address to write to
 * @param data Pointer to data to write
 * @param length Length of data to write
 * @return 0 on success, negative error code on failure
 */
int occp_write_data(const struct occp_backend *backend, uint64_t address, const uint8_t *data,
		    size_t length)
{
	struct occp_write_data_request write_req = {0};
	struct occp_header resp;
	int ret;

	if (length > OCCP_MAX_RW_SIZE) {
		LOG_ERR("OCCP write length %zu exceeds maximum %d", length, OCCP_MAX_RW_SIZE);
		return -EINVAL;
	}

	/* Issue a WRITE_DATA command */
	write_req.header.cmd_header.app_id = OCCP_APP_BASE;
	write_req.header.cmd_header.app_id = OCCP_APP_BASE;
	write_req.header.cmd_header.msg_id = OCCP_BASE_MSG_WRITE_DATA;
	write_req.header.cmd_header.length = sizeof(write_req) - sizeof(write_req.header)
					       + length;
	write_req.header.header_crc = crc8((uint8_t *)&write_req.header + 1,
				    sizeof(write_req.header) - 1, 0xD3, 0xFF, false);
	write_req.address_low = address & GENMASK(31, 0);
	write_req.address_high = (address >> 32);
	write_req.length = length;

	memcpy(occp_rw_buffer, &write_req, sizeof(write_req));
	memcpy(occp_rw_buffer + sizeof(write_req), data, length);
	ret = backend->send(backend, occp_rw_buffer, sizeof(write_req) + length);
	if (ret != 0) {
		LOG_ERR("Failed to send OCCP WRITE_DATA command: %d", ret);
		return ret;
	}
	/*
	 * TODO: this likely won't be required post-silicon, but for now the
	 * renode simulation framework requires a delay before writing new data
	 * since the remote SMC may not execute immediately after data is written
	 * to the I3C bus. If the response read starts early, the remote ROM
	 * will see an oversized write.
	 */
	k_msleep(100);
	/* Now read the response */
	ret = backend->receive(backend, (uint8_t *)&resp, sizeof(resp));
	if (ret != 0) {
		LOG_ERR("Failed to read OCCP WRITE_DATA response: %d", ret);
		return ret;
	}
	if (resp.cmd_header.flags) {
		LOG_ERR("OCCP WRITE_DATA command failed with flags: 0x%02x",
			resp.cmd_header.flags);
		return -EIO;
	}
	return 0;
}

/**
 * @brief Read data from OCCP device
 * @param backend OCCP backend to use
 * @param address Address to read from
 * @param data Pointer to buffer to store read data
 * @param length Length of data to read
 * @return 0 on success, negative error code on failure
 */
int occp_read_data(const struct occp_backend *backend, uint64_t address, uint8_t *data,
		   size_t length)
{
	struct occp_read_data_request read_req = {0};
	int ret;

	if (length > OCCP_MAX_RW_SIZE) {
		LOG_ERR("OCCP read length %zu exceeds maximum %d", length, OCCP_MAX_RW_SIZE);
		return -EINVAL;
	}

	/* Issue a READ_DATA command */
	read_req.header.cmd_header.app_id = OCCP_APP_BASE;
	read_req.header.cmd_header.msg_id = OCCP_BASE_MSG_READ_DATA;
	read_req.header.cmd_header.length = sizeof(read_req) - sizeof(read_req.header);
	read_req.header.header_crc = crc8((uint8_t *)&read_req.header + 1,
				   sizeof(read_req.header) - 1, 0xD3, 0xFF, false);
	read_req.address_low = address & GENMASK(31, 0);
	read_req.address_high = (address >> 32);
	read_req.length = length;

	ret = backend->send(backend, (uint8_t *)&read_req, sizeof(read_req));
	if (ret != 0) {
		LOG_ERR("Failed to send OCCP READ_DATA command: %d", ret);
		return ret;
	}
	/*
	 * TODO: this likely won't be required post-silicon, but for now the
	 * renode simulation framework requires a delay before writing new data
	 * since the remote SMC may not execute immediately after data is written
	 * to the I3C bus. If the response read starts early, the remote ROM
	 * will see an oversized write.
	 */
	k_msleep(100);

	/* Now read the response */
	ret = backend->receive(backend, occp_rw_buffer, sizeof(struct occp_header) + length);
	if (ret != 0) {
		LOG_ERR("Failed to read OCCP READ_DATA response: %d", ret);
		return ret;
	}
	memcpy(data, occp_rw_buffer + sizeof(struct occp_header), length);
	return 0;
}

/**
 * @brief Execute image at specified address
 * @param backend OCCP backend to use
 * @param execution_address Address to execute image from
 * @param cpu_id CPU ID to execute on
 * @return 0 on success, negative error code on failure
 */
int occp_execute_image(const struct occp_backend *backend, uint64_t execution_address, uint8_t cpu_id)
{
	struct occp_execute_image_request exec_req = {0};

	/* Issue an EXECUTE_IMAGE command */
	exec_req.header.cmd_header.app_id = OCCP_APP_BOOT;
	exec_req.header.cmd_header.msg_id = OCCP_BOOT_MSG_EXECUTE_IMAGE;
	exec_req.header.cmd_header.length = sizeof(exec_req) - sizeof(exec_req.header);
	exec_req.header.header_crc = crc8((uint8_t *)&exec_req.header + 1,
					sizeof(exec_req.header) - 1, 0xD3, 0xFF, false);
	exec_req.execution_address_low = execution_address & GENMASK(31, 0);
	exec_req.execution_address_high = (execution_address >> 32);
	exec_req.cpu_id = cpu_id;
	return backend->send(backend, (uint8_t *)&exec_req, sizeof(exec_req));
}
