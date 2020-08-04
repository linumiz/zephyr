/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
# include <net/socket.h>

void main(void)
{
	struct sockaddr_in addr;
	//struct in_addr addr;
	char ip[32] = {0};
	char *x;
	int rc;

	printk("Hello World! %s\n", CONFIG_BOARD);
	rc = net_addr_pton(AF_INET, "192.168.178.66", &addr.sin_addr);
	printk("rc: %d\n", rc);

	x = net_addr_ntop(AF_INET, &addr.sin_addr, ip, 32);
	printk("x: %p %s\n", x, ip);
}
