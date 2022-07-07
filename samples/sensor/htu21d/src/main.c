/*
 * Copyright (c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>

void main(void)
{
	int ret;
	struct sensor_value temp_value, hum_value;
	const struct device *dev = DEVICE_DT_GET_ANY(te_htu21d);
	
	__ASSERT(dev != NULL, "Failed to get device binding");
	__ASSERT(device_is_ready(dev), "Device %s is not ready", dev->name);
	printk("device name is %s\n", dev->name);
	
	/*temperature*/
	ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
	if (ret) {
		printk("sensor_sample_fetch failed ret %d\n", ret);
		return;
        }
	ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
	if (ret) {
		printk("sensor_channel_get failed ret %d\n", ret);
		return;
	}
	printk("temp is %d (%d micro)\n", temp_value.val1,
						temp_value.val2);
	k_sleep(K_MSEC(1000));

	/*humidity*/
	ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_HUMIDITY);
	if (ret) {
		printk("sensor_sample_fetch failed ret %d\n", ret);
		return;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum_value);
	if (ret) {
		printk("sensor_channel_get failed ret %d\n", ret);
		return;
	}

	printk("humidity is %d (%d micro)\n", hum_value.val1,
						hum_value.val2);
	k_sleep(K_MSEC(1000));
}


