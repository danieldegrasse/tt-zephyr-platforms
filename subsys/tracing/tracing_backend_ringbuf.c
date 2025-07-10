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


struct ring_tracing_buf {
	struct ring_buf rb;
	uint8_t data[CONFIG_RINGBUF_TRACING_BUFFER_SIZE];
};

struct ring_tracing_buf ring;

static void tracing_backend_ringbuf_output(
		const struct tracing_backend *backend,
		uint8_t *data, uint32_t length)
{
	if (ring_buf_space_get(&ring.rb) < length) {
		/* Not enough space in the ring buffer, drop the data */
		return;
	}

	ring_buf_put(&ring.rb, data, length);
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
