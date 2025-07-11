/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <tracing_core.h>
#include <tracing_buffer.h>
#include <tracing_backend.h>

/*
 * Ring buffer data structure. Host side
 * (debugger or other tool with memory access)
 * can read data from the get index of the ring buffer.
 * If the ring buffer is full, the host side should reset the ring buffer
 * by writing 0 to the put and get index, and clear the "full" flag.
 */

struct ring_tracing_buf {
	struct ring_buf rb;
	volatile uint8_t buffer_full;
	uint8_t data[CONFIG_RINGBUF_TRACING_BUFFER_SIZE];
};

struct ring_tracing_buf ring;

static void tracing_backend_ringbuf_output(
		const struct tracing_backend *backend,
		uint8_t *data, uint32_t length)
{
	if (ring.buffer_full) {
		/*
		 * We need to wait for the host to clear the full flag before
		 * we can reenable tracing support
		 */
		tracing_cmd_handle("disable", sizeof("enable"));
		while (ring.buffer_full) {
			/* Wait for the host to read data from the ring buffer */
			k_msleep(100);
		}
		tracing_cmd_handle("enable", sizeof("enable"));
		return;
	}
	if (ring_buf_space_get(&ring.rb) < length) {
		/* Buffer is full, can't stream new data */
		ring.buffer_full = 1;
	} else {
		ring_buf_put(&ring.rb, data, length);
	}
}

static void tracing_backend_ringbuf_init(void)
{
	ring_buf_init(&ring.rb, CONFIG_RINGBUF_TRACING_BUFFER_SIZE, ring.data);
}

const struct tracing_backend_api tracing_backend_ringbuf_api = {
	.init = tracing_backend_ringbuf_init,
	.output  = tracing_backend_ringbuf_output
};

TRACING_BACKEND_DEFINE(tracing_backend_ringbuf, tracing_backend_ringbuf_api);
