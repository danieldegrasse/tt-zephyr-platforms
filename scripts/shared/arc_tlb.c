/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

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

#define KB(n) (1024 * (n))
#define MB(n) (1024 * 1024 * (n))

#ifndef PAGE_SIZE
#define PAGE_SIZE KB(4)
#endif

#ifndef BIT
#define BIT(n) (1UL << (n))
#endif

#ifndef BIT_MASK
#define BIT_MASK(n) (BIT(n) - 1)
#endif

#define TENSTORRENT_PCI_VENDOR_ID 0x1e52


#define ARC_X 8
#define ARC_Y 0

#define TLB_2M_REG_SIZE            (3 * sizeof(uint32_t))
#define TLB_2M_SHIFT               21
#define TLB_2M_WINDOW_SIZE         BIT(TLB_2M_SHIFT)
#define TLB_2M_WINDOW_MASK         BIT_MASK(TLB_2M_SHIFT)
#define BH_2M_TLB_UC_DYNAMIC_START 190
#define BH_2M_TLB_UC_DYNAMIC_END   199
#define BH_NUM_2M_TLBS             202
#define BH_NUM_4G_TLBS             8
#define BH_NUM_TLBS                (BH_NUM_2M_TLBS + BH_NUM_4G_TLBS)

#define ARC_CSM_TLB 179

#define TLB_REGS_LEN PAGE_SIZE

#define ARC_CSM_BASE    0x10000000
#define TLB_CONFIG_ADDR 0x1FC00000

#define TENSTORRENT_IOCTL_MAGIC           0xFA
#define TENSTORRENT_IOCTL_GET_DEVICE_INFO _IO(TENSTORRENT_IOCTL_MAGIC, 0)
#define TENSTORRENT_IOCTL_GET_DRIVER_INFO _IO(TENSTORRENT_IOCTL_MAGIC, 5)
#define TENSTORRENT_IOCTL_ALLOCATE_TLB    _IO(TENSTORRENT_IOCTL_MAGIC, 11)
#define TENSTORRENT_IOCTL_FREE_TLB        _IO(TENSTORRENT_IOCTL_MAGIC, 12)
#define TENSTORRENT_IOCTL_CONFIGURE_TLB   _IO(TENSTORRENT_IOCTL_MAGIC, 13)

enum tlb_order {
	TLB_ORDER_RELAXED,        /* Unposted AXI Writes. Relaxed NOC ordering */
	TLB_ORDER_STRICT,         /* Unposted AXI Writes. Strict NOC ordering */
	TLB_ORDER_POSTED_RELAXED, /* Posted AXI Writes. Relaxed NOC ordering */
	TLB_ORDER_POSTED_STRICT,  /* Posted AXI Writes. Strict NOC ordering */
};

struct tenstorrent_get_device_info_inp {
	uint32_t output_size_bytes;
};

struct tenstorrent_get_device_info_out {
	uint32_t output_size_bytes;
	uint16_t vendor_id;
	uint16_t device_id;
	uint16_t subsystem_vendor_id;
	uint16_t subsystem_id;
	uint16_t bus_dev_fn;
	uint16_t max_dma_buf_size_log2;
	uint16_t pci_domain;
};

struct tenstorrent_get_device_info {
	struct tenstorrent_get_device_info_inp in;
	struct tenstorrent_get_device_info_out out;
};

struct tenstorrent_get_driver_info_inp {
	uint32_t output_size_bytes;
};

struct tenstorrent_get_driver_info_out {
	uint32_t output_size_bytes;
	/* IOCTL API version */
	uint32_t driver_version;
	uint8_t driver_version_major;
	uint8_t driver_version_minor;
	uint8_t driver_version_patch;
	uint8_t reserved0;
};

struct tenstorrent_get_driver_info {
	struct tenstorrent_get_driver_info_inp in;
	struct tenstorrent_get_driver_info_out out;
};

struct tenstorrent_allocate_tlb_inp {
	uint64_t size;
	uint64_t reserved;
};

struct tenstorrent_allocate_tlb_out {
	uint32_t id;
	uint32_t reserved0;
	uint64_t mmap_offset_uc;
	uint64_t mmap_offset_wc;
	uint64_t reserved1;
};

struct tenstorrent_allocate_tlb {
	struct tenstorrent_allocate_tlb_inp in;
	struct tenstorrent_allocate_tlb_out out;
};

struct tenstorrent_free_tlb_inp {
	uint32_t id;
};

struct tenstorrent_free_tlb_out {
};

struct tenstorrent_free_tlb {
	struct tenstorrent_free_tlb_inp in;
	struct tenstorrent_free_tlb_out out;
};

struct tenstorrent_noc_tlb_config {
	uint64_t addr;
	uint16_t x_end;
	uint16_t y_end;
	uint16_t x_start;
	uint16_t y_start;
	uint8_t noc;
	uint8_t mcast;
	uint8_t ordering;
	uint8_t linked;
	uint8_t static_vc;
	uint8_t reserved0[3];
	uint32_t reserved1[2];
};

struct tenstorrent_configure_tlb_inp {
	uint32_t id;
	struct tenstorrent_noc_tlb_config config;
};

struct tenstorrent_configure_tlb_out {
	uint64_t reserved;
};

struct tenstorrent_configure_tlb {
	struct tenstorrent_configure_tlb_inp in;
	struct tenstorrent_configure_tlb_out out;
};

struct tlb_data {
	int fd;
	const char *dev_name;
	uint16_t pci_device_id;
	volatile uint8_t *tlb;            /* 2MiB tlb window */
	uint32_t tlb_id;
};

static struct tlb_data _tlb_data;
static int verbose;

#define D(level, fmt, ...)                                                      \
	if (verbose >= level) {                                                 \
		printf("D: %s(): " fmt "\r\n", __func__, ##__VA_ARGS__);        \
	}

#define E(fmt, ...) fprintf(stderr, "E: %s(): " fmt "\r\n", __func__, ##__VA_ARGS__)

#define I(fmt, ...)                                                             \
	if (verbose >= 0) {                                                     \
		printf(fmt "\r\n", ##__VA_ARGS__);                              \
	}

static int program_noc(struct tlb_data *data, uint32_t x, uint32_t y, enum tlb_order order,
		       uint64_t phys, uint64_t *adjust)
{
	struct tenstorrent_configure_tlb tlb = {.in.id = data->tlb_id,
						.in.config = (struct tenstorrent_noc_tlb_config){
							.addr = phys & ~TLB_2M_WINDOW_MASK,
							.x_end = x,
							.y_end = y,
							.ordering = order,
						}};

	if (ioctl(data->fd, TENSTORRENT_IOCTL_CONFIGURE_TLB, &tlb) < 0) {
		E("ioctl(TENSTORRENT_IOCTL_GET_DRIVER_INFO): %s", strerror(errno));
		return -errno;
	}

	*adjust = phys & (uint64_t)TLB_2M_WINDOW_MASK;

	/* There isn't a new API for getting the current TLB programming */
	/* D(2, "tlb[%u]: %s", data->tlb_id, tlb2m2str(reg)); */
	D(2, "tlb[%u]: %lx", data->tlb_id, phys & ~TLB_2M_WINDOW_MASK);

	return 0;
}

static int open_tt_dev(struct tlb_data *data)
{

	if (data->fd >= 0) {
		/* already opened or not properly initialized */
		return 0;
	}

	data->fd = open(data->dev_name, O_RDWR);
	if (data->fd < 0) {
		E("%s: %s", strerror(errno), data->dev_name);
		return -errno;
	}

	D(1, "opened %s as fd %d", data->dev_name, data->fd);

	struct tenstorrent_get_device_info info = {
		.in.output_size_bytes = sizeof(struct tenstorrent_get_device_info_out),
	};

	if (ioctl(data->fd, TENSTORRENT_IOCTL_GET_DEVICE_INFO, &info) < 0) {
		E("ioctl(TENSTORRENT_IOCTL_GET_DEVICE_INFO): %s", strerror(errno));
		return -errno;
	}

	uint16_t vid = info.out.vendor_id;
	uint16_t did = info.out.device_id;
	uint8_t bus = info.out.bus_dev_fn >> 8;
	uint8_t dev = (info.out.bus_dev_fn >> 3) & 0x1f;
	uint8_t fun = info.out.bus_dev_fn & 0x07;

	D(1, "opened %04x:%04x %02x.%02x.%x", vid, did, bus, dev, fun);

	if (vid != TENSTORRENT_PCI_VENDOR_ID) {
		E("expected vendor id %04x (not %04x)", TENSTORRENT_PCI_VENDOR_ID, vid);
		return -ENODEV;
	}

	if (did != data->pci_device_id) {
		E("expected device id %04x (not %04x)", data->pci_device_id, did);
		return -ENODEV;
	}

	struct tenstorrent_get_driver_info driver_info = {
		.in.output_size_bytes = sizeof(struct tenstorrent_get_driver_info_out),
	};

	if (ioctl(data->fd, TENSTORRENT_IOCTL_GET_DRIVER_INFO, &driver_info) < 0) {
		E("ioctl(TENSTORRENT_IOCTL_GET_DRIVER_INFO): %s", strerror(errno));
		return -errno;
	}

	if (driver_info.out.driver_version < 2) {
		E("Need tlb allocation API requires at least driver version 2; have driver "
		  "version %d",
		  driver_info.out.driver_version);
		return -EFAULT;
	}

	return 0;
}

static void close_tt_dev(struct tlb_data *data)
{
	if (data->fd == -1) {
		/* not yet opened or already closed */
		return;
	}

	if (close(data->fd) < 0) {
		E("fd %d: %s", data->fd, strerror(errno));
		return;
	}

	D(1, "closed fd %d", data->fd);

	data->fd = -1;
}

/*
 * Map the 2MiB TLB window. This can remain mapped for the duration of the
 * application. We simply change where the TLB window points by writing to the TLB config
 * register.
 */
static int map_tlb(struct tlb_data *data)
{
	if (data->tlb != MAP_FAILED) {
		/* already mapped? cons improperly initialized? */
		return 0;
	}

	struct tenstorrent_allocate_tlb tlb = {.in.size = TLB_2M_WINDOW_SIZE};

	if (ioctl(data->fd, TENSTORRENT_IOCTL_ALLOCATE_TLB, &tlb) < 0) {
		E("ioctl(TENSTORRENT_IOCTL_ALLOCATE_TLB): %s", strerror(errno));
		return -errno;
	}

	data->tlb = mmap(NULL, TLB_2M_WINDOW_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, data->fd,
			 tlb.out.mmap_offset_uc);
	if (data->tlb == MAP_FAILED) {
		E("%s", strerror(errno));
		return -errno;
	}

	data->tlb_id = tlb.out.id;

	D(1, "mapped %zu@%08x to %zu@%p for 2MiB TLB window %d", (size_t)TLB_2M_WINDOW_SIZE,
	  (uint32_t)tlb.out.mmap_offset_uc, (size_t)TLB_2M_WINDOW_SIZE, data->tlb, data->tlb_id);

	return 0;
}

static int unmap_tlb(struct tlb_data *data)
{
	if (data->tlb == MAP_FAILED) {
		/* not currently mapped */
		return 0;
	}

	if (munmap((void *)data->tlb, TLB_2M_WINDOW_SIZE) < 0) {
		E("%s", strerror(errno));
		return -EFAULT;
	}

	D(1, "unmapped %zu@%p", (size_t)TLB_2M_WINDOW_SIZE, data->tlb);

	struct tenstorrent_free_tlb tlb = {.in.id = data->tlb_id};

	if (ioctl(data->fd, TENSTORRENT_IOCTL_FREE_TLB, &tlb) < 0) {
		E("ioctl(TENSTORRENT_IOCTL_ALLOCATE_TLB): %s", strerror(errno));
		return -errno;
	}

	data->tlb = MAP_FAILED;

	return 0;
}

int tlb_read(uint32_t addr, uint8_t *buf, size_t len)
{
	uint64_t adjust;
	int ret = program_noc(&_tlb_data, ARC_X, ARC_Y, TLB_ORDER_STRICT, addr, &adjust);

	if (ret) {
		E("failed to configure tlb to point to ARC addr %x", addr);
		return ret;
	}
	uint32_t *virt = (uint32_t *)(_tlb_data.tlb + adjust);

	D(2, "read from (%p,%p) (phys,virt)", (void *)(uintptr_t)addr, virt);
	memcpy(buf, virt, len);
	return 0;
}

int tlb_write(uint32_t addr, const uint8_t *buf, size_t len)
{
	uint64_t adjust;
	int ret = program_noc(&_tlb_data, ARC_X, ARC_Y, TLB_ORDER_STRICT, addr, &adjust);

	if (ret) {
		E("failed to configure tlb to point to ARC addr %x", addr);
		return ret;
	}
	uint32_t *virt = (uint32_t *)(_tlb_data.tlb + adjust);

	D(2, "write to (%p,%p) (phys,virt)", (void *)(uintptr_t)addr, virt);
	memcpy(virt, buf, len);
	return 0;
}

int tlb_init(void *data)
{
	int ret;
	struct tlb_init_data *init_data = (struct tlb_init_data *)data;

	_tlb_data.dev_name = init_data->dev_name;
	_tlb_data.pci_device_id = init_data->pci_device_id;
	_tlb_data.fd = -1;
	_tlb_data.tlb = MAP_FAILED;
	verbose = init_data->verbose;

	ret = open_tt_dev(&_tlb_data);
	if (ret < 0) {
		return ret;
	}

	ret = map_tlb(&_tlb_data);
	if (ret < 0) {
		return ret;
	}
	return ret;
}

void tlb_exit(void)
{
	unmap_tlb(&_tlb_data);
	close_tt_dev(&_tlb_data);
}
