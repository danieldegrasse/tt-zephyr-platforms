/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JLINK_H_
#define JLINK_H_

int jlink_init(int verbose, const char *serial_number);
void jlink_exit(void);

#endif /* JLINK_H_ */
