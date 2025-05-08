/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <libjaylink/libjaylink.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#define D(level, fmt, ...)                                                                         \
	if (verbose >= level) {                                                                    \
		printf("D: %s(): " fmt "\n", __func__, ##__VA_ARGS__);                             \
	}

#define E(fmt, ...) fprintf(stderr, "E: %s(): " fmt "\n", __func__, ##__VA_ARGS__)

#define I(fmt, ...)                                                                                \
	if (verbose >= 0) {                                                                        \
		printf(fmt "\n", ##__VA_ARGS__);                                                   \
	}

static struct jaylink_context *ctx;
static struct jaylink_device_handle *devh;
static int verbose;
static uint8_t caps[JAYLINK_DEV_EXT_CAPS_SIZE];
static struct jaylink_connection conn;

#define DIV_ROUND_UP(val, div) ((((val) + ((div) - 1))) / (div))

static int jlink_write_ir(uint8_t *data, int bit_len)
{
	uint8_t tms[DIV_ROUND_UP(bit_len, 8)];
	uint8_t	tdi = 0;
	uint8_t dummy_tdo[DIV_ROUND_UP(bit_len, 8)];
	int ret;

	/* Assume we start in run/test idle state */
	/* Move to SHIFT IR state */
	tms[0] = 0x3;
	ret = jaylink_jtag_io(devh, tms, &tdi, dummy_tdo, 4, JAYLINK_JTAG_VERSION_3);
	if (ret != JAYLINK_OK) {
		return ret;
	}

	memset(tms, 0, sizeof(tms));
	/* Set last bit of tms to 1 to exit from SHIFT IR state */
	tms[DIV_ROUND_UP(bit_len, 8) - 1] = 1 << ((bit_len - 1) % 8);
	/* Send IR data */
	ret = jaylink_jtag_io(devh, tms, data, dummy_tdo, bit_len,
			      JAYLINK_JTAG_VERSION_3);
	if (ret != JAYLINK_OK) {
		return ret;
	}
	/* Now in Exit1 IR, move back to idle state */
	memset(tms, 0, sizeof(tms));
	tms[0] = 0x1;
	ret = jaylink_jtag_io(devh, tms, &tdi, dummy_tdo, 2, JAYLINK_JTAG_VERSION_3);
	if (ret != JAYLINK_OK) {
		return ret;
	}

	return 0;
}

static int jlink_write_dr(uint8_t *data, int bit_len)
{
	uint8_t tms[DIV_ROUND_UP(bit_len, 8)];
	uint8_t	tdi = 0;
	uint8_t dummy_tdo[DIV_ROUND_UP(bit_len, 8)];
	int ret;

	/* Assume we start in run/test idle state */
	/* Move to SHIFT DR state */
	tms[0] = 0x1;
	ret = jaylink_jtag_io(devh, tms, &tdi, dummy_tdo, 3, JAYLINK_JTAG_VERSION_3);
	if (ret != JAYLINK_OK) {
		return ret;
	}

	memset(tms, 0, sizeof(tms));
	/* Set last bit of tms to 1 to exit from SHIFT DR state */
	tms[DIV_ROUND_UP(bit_len, 8) - 1] = 1 << ((bit_len - 1) % 8);
	/* Send DR data */
	ret = jaylink_jtag_io(devh, tms, data, dummy_tdo, bit_len,
			      JAYLINK_JTAG_VERSION_3);
	if (ret != JAYLINK_OK) {
		return ret;
	}
	/* Now in Exit1 DR, move back to idle state */
	memset(tms, 0, sizeof(tms));
	tms[0] = 0x1;
	ret = jaylink_jtag_io(devh, tms, &tdi, dummy_tdo, 2, JAYLINK_JTAG_VERSION_3);
	if (ret != JAYLINK_OK) {
		return ret;
	}

	return 0;
}

static int jlink_read_dr(uint8_t *out, int bit_len)
{
	uint8_t tms[DIV_ROUND_UP(bit_len, 8)];
	uint8_t tdi[DIV_ROUND_UP(bit_len, 8)];
	uint8_t dummy_tdo;
	int ret;

	/* Assume we start in run/test idle state */
	/* Move to SHIFT DR state */
	tms[0] = 0x1;
	tdi[0] = 0x0;
	ret = jaylink_jtag_io(devh, tms, tdi, &dummy_tdo, 3, JAYLINK_JTAG_VERSION_3);
	if (ret != JAYLINK_OK) {
		return ret;
	}

	memset(tms, 0, sizeof(tms));
	memset(tdi, 0, sizeof(tdi));
	/* Set last bit of tms to 1 to exit from SHIFT DR state */
	tms[DIV_ROUND_UP(bit_len, 8) - 1] = 1 << ((bit_len - 1) % 8);
	/* Read DR data */
	ret = jaylink_jtag_io(devh, tms, tdi, out, bit_len,
			      JAYLINK_JTAG_VERSION_3);
	if (ret != JAYLINK_OK) {
		return ret;
	}
	/* Now in Exit1 DR, move back to idle state */
	memset(tms, 0, sizeof(tms));
	tms[0] = 0x1;
	ret = jaylink_jtag_io(devh, tms, tdi, &dummy_tdo, 2, JAYLINK_JTAG_VERSION_3);
	if (ret != JAYLINK_OK) {
		return ret;
	}

	return 0;
}

static int jlink_go_idle(void)
{
	uint8_t tms = 0x1f;
	uint8_t tdi = 0;
	uint8_t tdo;
	int ret;

	/* Move to run/test idle state */
	ret = jaylink_jtag_io(devh, &tms, &tdi, &tdo, 6, JAYLINK_JTAG_VERSION_3);
	if (ret != JAYLINK_OK) {
		return ret;
	}

	return 0;
}


static void jlink_test(void)
{
	int ret;

	uint8_t idcode_ir = 0xC;
	uint32_t idcode = 0;

	ret = jlink_go_idle();
	if (ret < 0) {
		printf("Error, failed to enter run/test idle\n");
	}
	/* Write IR to read IDCODE */
	ret = jlink_write_ir(&idcode_ir, 4);
	if (ret < 0) {
		printf("Error, failed to write IDCODE reg\n");
	}
	/* Read IDCODE */
	ret = jlink_read_dr((uint8_t *)&idcode, 32);
	if (ret < 0) {
		printf("Error, failed to read idcode reg\n");
	}
	printf("Idcode reg was 0x%04X\n", idcode);
}

int jlink_init(int verbose, const char *serial_number)
{
	size_t num_devs;
	struct jaylink_device **devs;
	struct jaylink_device *selected_device = NULL;
	struct jaylink_connection conns[JAYLINK_MAX_CONNECTIONS];
	bool found_handle;
	size_t conn_count;
	int ret;

	ret = jaylink_init(&ctx);
	if (ret != JAYLINK_OK) {
		return -1;
	}

	ret = jaylink_discovery_scan(ctx, JAYLINK_HIF_USB);
	if (ret != JAYLINK_OK) {
		jaylink_exit(ctx);
		ctx = NULL;
		return -1;
	}

	ret = jaylink_get_devices(ctx, &devs, &num_devs);
	if (ret != JAYLINK_OK) {
		jaylink_exit(ctx);
		ctx = NULL;
		return -1;
	}

	if (serial_number) {
		uint32_t serial, test_serial;
		ret = jaylink_parse_serial_number(serial_number, &serial);
		if (ret != JAYLINK_OK) {
			E("Invalid serial number: %s", serial_number);
			goto error;
		}
		for (size_t i = 0; i < num_devs; i++) {
			ret = jaylink_device_get_serial_number(devs[i], &test_serial);
			if (ret != JAYLINK_OK) {
				E("Failed to get serial number for device %zu", i);
				goto error;
			}
			if (test_serial == serial) {
				selected_device = devs[i];
				I("Found JLink device with serial number: %s", serial_number);
				break;
			}
		}
		if (selected_device == NULL) {
			E("No JLink device found with serial number: %s", serial_number);
			goto error;
		}
	} else if (num_devs > 1) {
		I("Multiple JLink devices found, using the first one.");
		selected_device = devs[0];
	} else if (num_devs == 1) {
		selected_device = devs[0];
	} else {
		E("No JLink devices found.");
		goto error;
	}
	ret = jaylink_open(selected_device, &devh);
	if (ret != JAYLINK_OK) {
		E("Failed to open JLink device: %s", jaylink_strerror(ret));
		goto error;
	}
	jaylink_free_devices(devs, true);

	memset(caps, 0, JAYLINK_DEV_EXT_CAPS_SIZE);
	ret = jaylink_get_caps(devh, caps);
	if (ret != JAYLINK_OK) {
		E("Failed to get JLink capabilities: %s", jaylink_strerror(ret));
		goto error;
	}
	if (jaylink_has_cap(caps, JAYLINK_DEV_CAP_GET_EXT_CAPS)) {
		ret = jaylink_get_extended_caps(devh, caps);
		if (ret != JAYLINK_OK) {
			E("Failed to get extended capabilities: %s", jaylink_strerror(ret));
			goto error;
		}
	}

	if (jaylink_has_cap(caps, JAYLINK_DEV_CAP_REGISTER)) {
		conn.handle = 0;
		conn.pid = 0;
		strcpy(conn.hid, "0.0.0.0");
		conn.iid = 0;
		conn.cid = 0;

		ret = jaylink_register(devh, &conn, conns, &conn_count);
		if (ret != JAYLINK_OK) {
			E("jaylink_register() failed: %s", jaylink_strerror(ret));
			goto error;
		}

		found_handle = false;

		for (size_t i = 0; i < conn_count; i++) {
			if (conns[i].handle == conn.handle) {
				found_handle = true;
				break;
			}
		}
		if (!found_handle) {
			E("Maximum number of JLink connections reached");
			ret = -ENODEV;
			goto error;
		}
	}

	/* Select JTAG interface */
	if (jaylink_has_cap(caps, JAYLINK_DEV_CAP_SELECT_TIF)) {
		ret = jaylink_select_interface(devh, JAYLINK_TIF_JTAG, NULL);
		if (ret != JAYLINK_OK) {
			E("jaylink_select_interface() failed: %s", jaylink_strerror(ret));
			goto error;
		}
	}

	jlink_test();
	return 0;
error:
	if (devh) {
		jaylink_close(devh);
		devh = NULL;
	}
	jaylink_free_devices(devs, true);
	jaylink_exit(ctx);
	ctx = NULL;
	return ret;
}

void jlink_exit(void)
{
	size_t conn_count;
	struct jaylink_connection conns[JAYLINK_MAX_CONNECTIONS];

	if (devh) {
		if (jaylink_has_cap(caps, JAYLINK_DEV_CAP_REGISTER)) {
			jaylink_unregister(devh, &conn, conns, &conn_count);
		}
		jaylink_close(devh);
		devh = NULL;
	}
	if (ctx) {
		jaylink_exit(ctx);
		ctx = NULL;
	}
}
