/*
 * Copyright (c) 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <stdlib.h>

static void print_stack_usage(const char *name, k_tid_t tid,
                              size_t total_stack_bytes)
{
	size_t unused = 0;
	int rc = k_thread_stack_space_get(tid, &unused);
	if (rc == 0) {
		size_t used = total_stack_bytes - unused;
		printk("[%s] stack: used=%u bytes, unused=%u, total=%u\n",
				name ? name : "?", (unsigned)used, (unsigned)unused,
				(unsigned)total_stack_bytes);
	} else {
		printk("[%s] k_thread_stack_space_get failed: %d\n",
				name ? name : "?", rc);
	}
}

K_MUTEX_DEFINE(size_check);
int main(void)
{
	void *mem = malloc(1);
	free(mem);

	k_msleep(1000);
	print_stack_usage("sleep", k_current_get(), CONFIG_MAIN_STACK_SIZE);

	k_mutex_lock(&size_check, K_FOREVER);
	print_stack_usage("mutex_lock", k_current_get(), CONFIG_MAIN_STACK_SIZE);
	k_mutex_unlock(&size_check);
	print_stack_usage("mutex_unlock", k_current_get(), CONFIG_MAIN_STACK_SIZE);

	return 0;
}
