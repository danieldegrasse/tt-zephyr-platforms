/*
 * Copyright (c) 2025 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i3c.h>

#include <zephyr/drivers/misc/tt_smc_remoteproc.h>

#include <soc.h>

const struct device *smc_dev = DEVICE_DT_GET(DT_NODELABEL(remoteproc));

uint8_t remote_smc_bin[] = {
	#include CONFIG_REMOTE_SMC_BINARY_HEADER
};
const unsigned int remote_smc_bin_len = sizeof(remote_smc_bin);

int main(void)
{
	int ret;

	printf("Primary BL1 is running!\n");

	if (!device_is_ready(smc_dev)) {
		printf("Remote SMC device not ready\n");
		return -ENODEV;
	}

	ret = tt_smc_remoteproc_boot(smc_dev, 0xc0066000, remote_smc_bin, remote_smc_bin_len);
	if (ret != 0) {
		printf("Failed to boot remote SMC: %d\n", ret);
		return ret;
	}

	while (1) {
		k_sleep(K_SECONDS(1));
		printf("Primary BL1 is alive!\n");
	}

	return 0;
}
