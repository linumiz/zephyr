/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/util.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/fuel_gauge.h>

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(ht_hy4245);
	int ret = 0;

	if (dev == NULL) {
		printk("\nError: no device found.\n");
		return 0;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return 0;
	}

	printk("Found device \"%s\", getting fuel gauge data\n", dev->name);

	if (dev == NULL) {
		return 0;
	}

	while (1) {

		fuel_gauge_prop_t props[] = {
			FUEL_GAUGE_RUNTIME_TO_EMPTY,
			FUEL_GAUGE_RUNTIME_TO_FULL,
			FUEL_GAUGE_VOLTAGE,
			FUEL_GAUGE_REMAINING_CAPACITY,
			FUEL_GAUGE_CHARGE_VOLTAGE,
		};

		union fuel_gauge_prop_val vals[ARRAY_SIZE(props)];

		ret = fuel_gauge_get_props(dev, props, vals, ARRAY_SIZE(props));
		if (ret < 0) {
			printk("Error: cannot get properties\n");
		} else {
			printk("Time to empty %d minutes\n", vals[0].runtime_to_empty);

			printk("Time to full %d minutes\n", vals[1].runtime_to_full);

			printk("Voltage %d\n uV", vals[2].voltage);

			printk("Remaning capacity %u\n", vals[3].remaining_capacity);

			printk("Charge voltge %u\n uV", vals[4].chg_voltage);
		}

		k_sleep(K_MSEC(5000));
	}
	return 0;
}
