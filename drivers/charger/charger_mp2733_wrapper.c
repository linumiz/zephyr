/*
 * Copyright 2025 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define MP2733  DT_NODELABEL(charger)

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/charger.h>

static const struct device *const dev = DEVICE_DT_GET(MP2733);

int set_property(enum charger_property prop, union charger_propval *val){

	const struct charger_driver_api * api = (const struct charger_driver_api *)dev->api;
	int ret;

	ret = api->set_property(dev, prop, val);
        if(ret){
		printk("\nwset err : %d\n", ret);
		return ret;
	}
	return 0;
}

int get_property(enum charger_property prop, union charger_propval *val){
	const struct charger_driver_api * api = (const struct charger_driver_api *)dev->api;
	int ret;

	ret = api->get_property(dev, prop, val);
	if(ret){
		printk("\nwget err : %d\n",ret);
		return ret;
	}
	return 0;
}

int charge_enable(const bool enable){
	const struct charger_driver_api * api = (const struct charger_driver_api *)dev->api;
	int ret;

	ret = api->charge_enable(dev, enable);
	if(ret){
		printk("\nwchrg enable err : %d\n", ret);
		return ret;
	}
	return 0;
}
