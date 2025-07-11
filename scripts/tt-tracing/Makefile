# Copyright (c) 2025 Tenstorrent Inc.
# SPDX-License-Identifier: Apache-2.0

SOURCES=tracing.c ../shared/arc_tlb.c

CFLAGS += -Wall -Werror -Wextra -O0 -g -std=gnu11 -I../../include -I../shared

tt-tracing: $(SOURCES)
	$(CC) $(CFLAGS) -o $@ $^


.PHONY: clean

clean:
	rm -rf tt-tracing *.o
