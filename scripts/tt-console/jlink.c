/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <libjaylink/libjaylink.h>
#include <stdio.h>

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

int jlink_init(int verbose, const char *serial_number)
{
	size_t num_devs;
	struct jaylink_device **devs;
	struct jaylink_device *selected_device = NULL;
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
	return 0;
error:
	jaylink_free_devices(devs, true);
	jaylink_exit(ctx);
	ctx = NULL;
	return ret;
}

void jlink_exit(void)
{
	if (devh) {
		jaylink_close(devh);
		devh = NULL;
	}
	if (ctx) {
		jaylink_exit(ctx);
		ctx = NULL;
	}
}
