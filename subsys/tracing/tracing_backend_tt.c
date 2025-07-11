/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
/* Defines the scratch register to write ringbuf address to */
#include <status_reg.h>

#define TRACE_MAGIC 0x54524143 /* "TRAC" in ASCII */

extern struct ring_tracing_buf ring;

struct tt_tracing_data {
	uint32_t magic;
	struct ring_tracing_buf *ring;
} trace_data;

static int tracing_backend_tt_init(void)
{
	trace_data.magic = TRACE_MAGIC;
	trace_data.ring = &ring;

	sys_write32(((uint32_t) &trace_data), CMFW_TRACE_BUF_REG_ADDR);
	return 0;
}

SYS_INIT(tracing_backend_tt_init, APPLICATION, 0);
