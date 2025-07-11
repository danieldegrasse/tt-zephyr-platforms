/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR__
#include "attrs.x"
#endif

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <libgen.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <termios.h>
#include <sys/time.h>
#include <unistd.h>

#include "arc_tlb.h"

/* The limit is used by algorithm for distinguishing between empty and full
 * state.
 */
#ifdef CONFIG_RING_BUFFER_LARGE
typedef uint32_t ring_buf_idx_t;
#define RING_BUFFER_MAX_SIZE (UINT32_MAX / 2)
#else
typedef uint16_t ring_buf_idx_t;
#define RING_BUFFER_MAX_SIZE (UINT16_MAX / 2)
#endif

struct ring_buf_index { ring_buf_idx_t head, tail, base; };

/** @endcond */

/**
 * @brief A structure to represent a ring buffer
 */
struct ring_buf {
	uint32_t buffer_addr;
	struct ring_buf_index put;
	struct ring_buf_index get;
	uint32_t size;
};

#ifndef TRACE_DISCOVERY_ADDR
#define TRACE_DISCOVERY_ADDR 0x80030458
#endif

#define STATUS_POST_CODE_REG_ADDR 0x80030060
#define POST_CODE_PREFIX          0xc0de

#define BH_SCRAPPY_PCI_DEVICE_ID  0xb140

#define TT_DEVICE "/dev/tenstorrent/0"

/* Definition of functions used to access memory */
struct mem_access_driver {
	int (*start)(void *init_data);
	int (*read)(uint32_t addr, uint8_t *buf, size_t len);
	int (*write)(uint32_t addr, const uint8_t *buf, size_t len);
	void (*stop)(void);
};

static int verbose;

static struct tlb_init_data tlb_init_data = {
	.dev_name = TT_DEVICE,
	.pci_device_id = BH_SCRAPPY_PCI_DEVICE_ID,
	.tlb_id = BH_2M_TLB_UC_DYNAMIC_START + 1,
};

#define D(level, fmt, ...)                                                      \
	if (verbose >= level) {                                                 \
		printf("D: %s(): " fmt "\r\n", __func__, ##__VA_ARGS__);        \
	}

#define E(fmt, ...) fprintf(stderr, "E: %s(): " fmt "\r\n", __func__, ##__VA_ARGS__)

#define I(fmt, ...)                                                             \
	if (verbose >= 0) {                                                     \
		printf(fmt "\r\n", ##__VA_ARGS__);                              \
	}

static int check_post_code(struct mem_access_driver *driver)
{
	union {
		struct {
			uint16_t code: 14;
			uint8_t id: 2;
			uint16_t prefix;
		};
		uint32_t data;
	} val = {
		.data = -1,
	};

	_Static_assert(sizeof(val) == sizeof(uint32_t), "invalid size of val");

	if (driver->read(STATUS_POST_CODE_REG_ADDR, (uint8_t *)&val.data, sizeof(val.data)) < 0) {
		E("failed to read post code");
		return -EIO;
	}
	if (val.prefix != POST_CODE_PREFIX) {
		E("prefix 0x%04x does not match expected prefix 0x%04x", val.prefix,
		  POST_CODE_PREFIX);
		return -EINVAL;
	}

	D(2, "POST code: (%04x, %02x, %04x)", val.prefix, val.id, val.code);

	return 0;
}

static uint8_t read_buf[4096];
static bool running;

static void handler(int sig)
{
	I("\nCaught signal %d (%s)", sig, strsignal(sig));
	running = false;
}

static int loop(struct mem_access_driver *driver)
{
	int ret;
	uint32_t ring_buf_addr;
	struct ring_buf ring_buf;
	uint32_t read_len;
	FILE *fp;

	running = true;

	if (signal(SIGINT, handler) == SIG_ERR) {
		E("signal: %s", strerror(errno));
		return EXIT_FAILURE;
	}

	fp = fopen("trace_data", "wb");
	if (!fp) {
		E("failed to open trace_data");
		return -EIO;
	}

	ret = check_post_code(driver);
	if (ret < 0) {
		goto out;
	}

	if (driver->read(TRACE_DISCOVERY_ADDR, (uint8_t *)&ring_buf_addr,
			 sizeof(ring_buf_addr)) < 0) {
		E("failed to read ring buffer address");
		return -EIO;
	}
	if (ring_buf_addr >> 24 != 0xCA) {
		E("invalid ring buffer address 0x%08x", ring_buf_addr);
		return -EINVAL;
	}
	/* Mask out the prefix, convert to address in CSM */
	ring_buf_addr = (ring_buf_addr & 0xFFFFFF) + 0x10000000;
	I("ring buffer address: 0x%08x", ring_buf_addr);
	while (running) {
		if (driver->read(ring_buf_addr, (uint8_t *)&ring_buf, sizeof(ring_buf)) < 0) {
			E("failed to read ring buffer descriptor");
			return -EIO;
		}

		if (ring_buf.get.base <= ring_buf.put.base) {
			/* Read up to put base */
			read_len = ring_buf.put.base - ring_buf.get.base;
		} else {
			/* Read to end of buffer */
			read_len = ring_buf.size - ring_buf.get.base;
		}
		I("ring buffer get base: %u, put base: %u, read_len: %u",
		  ring_buf.get.base, ring_buf.put.base, read_len);
		if (read_len > sizeof(read_buf)) {
			E("read_len %u is larger than read buffer size %zu", read_len,
			  sizeof(read_buf));
			return -EINVAL;
		}
		if (driver->read(ring_buf.buffer_addr + ring_buf.get.base, read_buf, read_len) < 0) {
			E("failed to read ring buffer data");
			return -EIO;
		}
		ring_buf.get.base += read_len;
		ring_buf.get.tail += read_len;
		ring_buf.get.head += read_len;
		/* Update ring buffer */
		if (driver->write(ring_buf_addr, (uint8_t *)&ring_buf, sizeof(ring_buf)) < 0) {
			E("failed to write ring buffer descriptor");
			return -EIO;
		}
		if (read_len > 0) {
			I("read %u bytes from ring buffer", read_len);
		}
		fwrite(read_buf, 1, read_len, fp);
	}

out:
	fclose(fp);
	driver->stop();
	return ret;
}

static void usage(const char *progname)
{
	I("Firmware Tracing application for use with Tenstorrent PCIe cards\n"
	  "Copyright (c) 2025 Tenstorrent AI ULC\n"
	  "\n"
	  "\n"
	  "%s: %s [args..]\n"
	  "\n"
	  "args:\n"
	  "-d <path>          : path to device node (default: %s)\n"
	  "-h                 : print this help message\n"
	  "-i <pci_device_id> : pci device id (default: %04x)\n"
	  "-q                 : decrease debug verbosity\n"
	  "-v                 : increase debug verbosity\n",
	  __func__, progname, TT_DEVICE, BH_SCRAPPY_PCI_DEVICE_ID);
}

static int parse_args(int argc, char **argv)
{
	int c;

	while ((c = getopt(argc, argv, "d:hi:qv")) != -1) {
		switch (c) {
		case 'd':
			tlb_init_data.dev_name = optarg;
			break;
		case 'h':
			usage(basename(argv[0]));
			exit(EXIT_SUCCESS);
		case 'i': {
			long pci_device_id;

			errno = 0;
			pci_device_id = strtol(optarg, NULL, 0);
			if ((pci_device_id < 0) || (pci_device_id > UINT16_MAX)) {
				errno = ERANGE;
			}
			if (errno != 0) {
				E("invalid operand to -i %s: %s", optarg, strerror(errno));
				usage(basename(argv[0]));
				return -errno;
			}
			tlb_init_data.pci_device_id = (uint16_t)pci_device_id;
		} break;
		case 'q':
			--verbose;
			break;
		case 'v':
			++verbose;
			break;
		case ':':
			E("option -%c requires an operand\n", optopt);
			usage(basename(argv[0]));
			return -EINVAL;
		case '?':
			E("unrecognized option -%c\n", optopt);
			usage(basename(argv[0]));
			return -EINVAL;
		}
	}

	/* perform extra checking here and error as needed */

	return 0;
}

struct mem_access_driver tlb_driver = {
	.start = tlb_init,
	.read = tlb_read,
	.write = tlb_write,
	.stop = tlb_exit,
};

int main(int argc, char **argv)
{
	struct mem_access_driver *driver;
	void *init_data;

	if (parse_args(argc, argv) < 0) {
		return EXIT_FAILURE;
	}

	tlb_init_data.verbose = verbose;
	init_data = &tlb_init_data;
	driver = &tlb_driver;

	if (driver->start(init_data) < 0) {
		goto error;
	}

	if (loop(driver) < 0) {
		goto error;
	}

	return EXIT_SUCCESS;

error:
	driver->stop();
	return EXIT_FAILURE;
}
