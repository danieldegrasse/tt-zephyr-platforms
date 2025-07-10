# Copyright (c) 2025 Tenstorrent AI ULC
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_TRACING_BACKEND_RINGBUF tracing_backend_ringbuf.c)
zephyr_sources_ifdef(CONFIG_TRACING_BACKEND_TT tracing_backend_tt.c)
