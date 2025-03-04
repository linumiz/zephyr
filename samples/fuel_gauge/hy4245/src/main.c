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
	uint8_t data[32] = {0};

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

	ret = hy4245_enable_flash_access(dev);
	if (ret) {
		printk("enable flash access err %d\n", ret);
		return 0;
	}

	ret = hy4245_set_calibration_mode(dev);
	if (ret) {
		printk("set calib mode err %d\n", ret);
		return 0;
	}

	ret =hy4245_access_flash_data(dev, 0x3, 1,
			data, sizeof(data), true);
	if (ret) {
		printk("read flash data err %d\n", ret);
		return 0;
	}

	printk("read flash data\n", ret);
	for(int i = 0; i<sizeof(data); i++)
		printk("%x\t", data[i]);

#if 0
	data[5] = 0xfa;

	ret =hy4245_access_flash_data(dev, 0x3, 1,
			data, sizeof(data), false);
	if (ret) {
		printk("write flash data err %d\n", ret);
		return 0;
	}
#endif

	while (1) {
		fuel_gauge_prop_t props[] = {
			FUEL_GAUGE_RUNTIME_TO_EMPTY,
			FUEL_GAUGE_RUNTIME_TO_FULL,
			FUEL_GAUGE_VOLTAGE,
			FUEL_GAUGE_REMAINING_CAPACITY,
			FUEL_GAUGE_FULL_CHARGE_CAPACITY,
			FUEL_GAUGE_CHARGE_VOLTAGE,
			FUEL_GAUGE_CHARGE_CURRENT,
		};

		union fuel_gauge_prop_val vals[ARRAY_SIZE(props)];

		memset(vals, 0, sizeof(vals));
		ret = fuel_gauge_get_props(dev, props, vals, ARRAY_SIZE(props));
		if (ret < 0) {
			printk("Error: cannot get properties\n");
		} else {
			printk("Time to empty %d minutes\n", vals[0].runtime_to_empty);

			printk("Time to full %d minutes\n", vals[1].runtime_to_full);

			printk("Voltage %d uV\n", vals[2].voltage);

			printk("Remaning capacity %u uAh\n", vals[3].remaining_capacity);
			printk("full capacity %u uAh\n", vals[4].full_charge_capacity);

			printk("Charge voltge %u uV\n", vals[5].chg_voltage);
			printk("Charge voltge %u uA\n", vals[6].chg_current);
		}

		k_sleep(K_MSEC(5000));
	}
	return 0;
}
