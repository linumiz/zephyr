/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if 1
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mp2733_charger.h>

int main(void)
{
	union charger_propval val;
	int ret;

	val.input_current_regulation_current_ua = 5000000;
	ret = set_property(CHARGER_PROP_INPUT_REGULATION_CURRENT_UA, &val);
	if(ret){
		printk("\n1. set err\n");
	}

	ret = get_property(CHARGER_PROP_INPUT_REGULATION_CURRENT_UA, &val);
	if(ret){
		printk("\n2. get err : %d\n", ret);
	}

	ret = charge_enable(0);
	if(ret){
		printk("\n3. chrg disble err: %d\n", ret);
	}
	
	ret = charge_enable(1);
	if(ret){
		printk("\n3. chrg enable err: %d\n", ret);
	}

	return 0;
}
#endif

