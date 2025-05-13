/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARC_JTAG_H_
#define ARC_JTAG_H_

int arc_jtag_init(int verbose, const char *serial_number);
void arc_jtag_exit(void);

#endif /* ARC_JTAG_H_ */
