/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/kernel.h>

#define COMP_NODE DT_ALIAS(comp)

static void comp_callback(const struct device *dev, void *user_data)
{
	printk("Comparator triggered! Output: %d\n",
	       comparator_get_output(dev));
}

int main(void)
{
	const struct device *comp = DEVICE_DT_GET(COMP_NODE);
	int ret;

	if (!device_is_ready(comp)) {
		printk("Comparator not ready\n");
		return -ENODEV;
	}

	ret = comparator_set_trigger_callback(comp, comp_callback, NULL);
	if (ret < 0) {
		printk("Failed to set callback: %d\n", ret);
		return ret;
	}

	ret = comparator_set_trigger(comp, COMPARATOR_TRIGGER_RISING_EDGE);
	if (ret < 0) {
		printk("Failed to set trigger: %d\n", ret);
		return ret;
	}

	return 0;
}
