/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
/* Defines the scratch register to write ringbuf address to */
#include <status_reg.h>

extern volatile struct ring_tracing_buf ring;

static int tracing_backend_tt_init(void)
{
	sys_write32((uint32_t)(uintptr_t)&ring, CMFW_TRACE_BUF_REG_ADDR);
	return 0;
}

SYS_INIT(tracing_backend_tt_init, APPLICATION, 0);
