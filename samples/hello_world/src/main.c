/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>

#define DELAY (12000000)

int main(void)
{
//	while (true) {
		printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
//		printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
//		delay_cycles(DELAY);
//		k_msleep(1000);
//	}

	return 0;
}
