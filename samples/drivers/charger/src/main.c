/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/charger.h>

#define MP2733 	DT_NODELABEL(charger)

static const struct device *const dev = DEVICE_DT_GET(MP2733);

int main(void)
{
	printk("main\n");
	const struct charger_driver_api * api = (const struct charger_driver_api *)dev->api;
	union charger_propval val;
	int ret;
	ret = api->get_property(dev, CHARGER_PROP_INPUT_REGULATION_CURRENT_UA, &val);
	if(ret)
	{
		printk("\n1. get err : %d\n", ret);
	}
	
	ret = api->set_property(dev, CHARGER_PROP_INPUT_REGULATION_CURRENT_UA, &val);
	if(ret)
	{
		printk("\n2. set err : %d\n", ret);
	}
	
	ret = api->get_property(dev, CHARGER_PROP_INPUT_REGULATION_CURRENT_UA, &val);
	if(ret)
	{
		printk("\n3. get err : %d\n", ret);
	}

	return 0;
}
