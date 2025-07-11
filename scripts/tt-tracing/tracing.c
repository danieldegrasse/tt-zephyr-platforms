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
#include <libgen.h>
#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

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

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

struct ring_buf_index {
	ring_buf_idx_t head;
	ring_buf_idx_t tail;
	ring_buf_idx_t base;
};

/**
 * @brief A structure to represent a ring buffer
 */
struct ring_buf {
	uint32_t buffer_addr;
	struct ring_buf_index put;
	struct ring_buf_index get;
	uint32_t size;
};

struct ring_tracing_buf {
	struct ring_buf rb;
	volatile uint8_t buffer_full;
};

struct tt_tracing_data {
	uint32_t magic;
	uint32_t ring_buf_addr; /* Address of the ring tracing buffer */
};

#ifndef TRACE_DISCOVERY_ADDR
#define TRACE_DISCOVERY_ADDR 0x80030458
#endif

#ifndef TRACE_MAGIC
#define TRACE_MAGIC 0x54524143 /* "TRAC" in ASCII */
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
		return -EINVAL;
	}

	D(2, "POST code: (%04x, %02x, %04x)", val.prefix, val.id, val.code);

	return 0;
}

static int check_tracing_magic(struct tt_tracing_data *trace_data)
{
	if (trace_data->magic != TRACE_MAGIC) {
		E("magic 0x%08x does not match expected magic 0x%08x", trace_data->magic,
		  TRACE_MAGIC);
		return -EINVAL;
	}
	D(2, "Tracing magic: 0x%08x", trace_data->magic);

	return 0;
}

static int ring_buf_get(struct mem_access_driver *driver, uint32_t ring_buf_addr,
			uint8_t *data, uint32_t size)
{
	int ret;
	struct ring_buf ring_buf;
	struct ring_buf_index get;
	ring_buf_idx_t head_offset, available, wrap_size, tail_offset;

	ret = driver->read(ring_buf_addr, (uint8_t *)&ring_buf, sizeof(ring_buf));
	if (ret < 0) {
		E("failed to read ring buffer descriptor");
		return ret;
	}
	/* Check how many bytes are available in the ring buffer */
	available = ring_buf.put.head - ring_buf.get.tail;
	if (available == 0) {
		D(2, "Ring buffer is empty");
		return 0;
	}
	size = MIN(size, available);
	/*
	 * Copy ring buffer get index. This way we won't modify the put index,
	 * which the device could be updating
	 */
	memcpy(&get, &ring_buf.get, sizeof(get));
	/* Calculate the head offset, and pull data from the buffer */
	head_offset = get.head - get.base;
	if (head_offset > ring_buf.size) {
		/* Ring base hasn't been updated yet */
		head_offset -= ring_buf.size;
	}
	wrap_size = ring_buf.size - head_offset;
	size = MIN(size, wrap_size);
	D(2, "Reading %u bytes from ring buffer at offset %u", size, head_offset);
	/* Read data from the ring buffer */
	ret = driver->read(ring_buf.buffer_addr + head_offset, data, size);
	if (ret < 0) {
		E("failed to read ring buffer data");
		return ret;
	}
	/* Update the ring buffer get index */
	get.tail += size;
	get.head = get.tail;
	tail_offset = get.tail - get.base;
	if (tail_offset >= ring_buf.size) {
		/* We wrapped: adjust ring base */
		get.base += ring_buf.size;
	}
	D(1, "Updated ring buffer get index: head=%u, tail=%u, base=%u",
	   get.head, get.tail, get.base);
	/* Write the updated ring buffer get index */
	ret = driver->write(ring_buf_addr + offsetof(struct ring_buf, get),
			    (uint8_t *)&get, sizeof(get));
	if (ret < 0) {
		E("failed to write ring buffer descriptor");
		return ret;
	}
	return size;
}

static uint8_t read_buf[4096];
static bool running;
static char *output_file;

static void handler(int sig)
{
	I("\nCaught signal %d (%s)", sig, strsignal(sig));
	running = false;
}

static int loop(struct mem_access_driver *driver)
{
	int ret;
	uint32_t trace_data_addr, ring_buf_addr;
	struct ring_tracing_buf ring_buf;
	struct tt_tracing_data trace_data;
	uint32_t read_count = 0;
	int size;
	FILE *fp;

	running = true;

	if (signal(SIGINT, handler) == SIG_ERR) {
		E("signal: %s", strerror(errno));
		return EXIT_FAILURE;
	}

	fp = fopen(output_file, "wb");
	if (!fp) {
		E("failed to open %s", output_file);
		return -EIO;
	}

	ret = check_post_code(driver);
	if (ret < 0) {
		I("Waiting for post code to be set by the firmware");
	}
	while (running && (ret < 0)) {
		ret = check_post_code(driver);
	}

	ret = driver->read(TRACE_DISCOVERY_ADDR, (uint8_t *)&trace_data_addr,
			sizeof(trace_data_addr));
	if (ret < 0) {
		E("failed to read tracing discovery address");
		ret = -EIO;
		goto out;
	}
	if (trace_data_addr == 0) {
		E("tracing discovery address is 0, tracing is not enabled");
		ret = -ENODEV;
		goto out;
	}
	ret = driver->read(trace_data_addr, (uint8_t *)&trace_data, sizeof(trace_data));
	if (ret < 0) {
		E("failed to read tracing data");
		goto out;
	}

	ret = check_tracing_magic(&trace_data);
	if (ret < 0) {
		goto out;
	}

	ring_buf_addr = trace_data.ring_buf_addr + offsetof(struct ring_tracing_buf, rb);
	ret = driver->read(ring_buf_addr, (uint8_t *)&ring_buf, sizeof(ring_buf));
	if (ret < 0) {
		E("failed to read ring buffer descriptor");
		goto out;
	}
	if (ring_buf.buffer_full) {
		I("Ring buffer is full, resetting it");
		ring_buf.rb.put.head = ring_buf.rb.put.tail = ring_buf.rb.put.base = 0;
		ring_buf.rb.get.head = ring_buf.rb.get.tail = ring_buf.rb.get.base = 0;
		ring_buf.buffer_full = 0;
		ret = driver->write(ring_buf_addr, (uint8_t *)&ring_buf, sizeof(ring_buf));
		if (ret < 0) {
			E("failed to write ring buffer descriptor");
			goto out;
		}
	}

	while (running) {
		size = ring_buf_get(driver, ring_buf_addr, read_buf, sizeof(read_buf));
		if (size < 0) {
			E("Failed to read ring buffer");
			ret = size;
			goto out;
		}
		read_count += size;
		/* Print a new count so the user knows we are making progress */
		printf("\rRead %u bytes", read_count);
		ret = fwrite(read_buf, 1, size, fp);
		if (ret != size) {
			E("Failed to write %u bytes to %s: %s", size, output_file,
			  strerror(errno));
			ret = -EIO;
			goto out;
		}
	}

out:
	I("\r\nRead %u bytes from tracing ring buffer", read_count);
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
	  "%s: %s <output_file> [args..]\n"
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
	if (optind >= argc) {
        	E("Missing argument for filename");
		usage(basename(argv[0]));
		return -EINVAL;
    	}

	output_file = argv[optind];

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
