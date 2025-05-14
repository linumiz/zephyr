/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>

int main(void)
{
	while (true) {
		printf("Hello World! %s %llu\n", CONFIG_BOARD_TARGET, k_uptime_get());
		k_msleep(1000);
	}

	return 0;
}
