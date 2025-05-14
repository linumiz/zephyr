/*
 * Copyright (c) 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>

int main(void)
{
	while (1) {
		printk("Hello World! %s %" PRIu32 "\n", CONFIG_BOARD_TARGET, k_cycle_get_32());
		k_msleep(1000);
	}
	return 0;
}
