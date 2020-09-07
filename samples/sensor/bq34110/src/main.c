/*
 * Copyright (c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apachw-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <sys/printk.h>

static void bq34110_show_values(const char *type, struct sensor_value value)
{
	if ((value.val2 < 0) && (value.val1 >= 0)) {
		value.val2 = -(value.val2);
		printk("%s: -%d.%06d\n", type, value.val1, value.val2);
	} else if ((value.val2 > 0) && (value.val1 < 0)) {
		printk("%s: %d.%06d\n", type, value.val1, value.val2);
	} else if ((value.val2 < 0) && (value.val1 < 0)) {
		value.val2 = -(value.val2);
		printk("%s: %d.%06d\n", type, value.val1, value.val2);
	} else {
		printk("%s: %d.%06d\n", type, value.val1, value.val2);
	}
}

static void do_main(const struct device *dev)
{
	int ret = 0, observation = 0;
	struct sensor_value voltage, current, state_of_charge,
	       full_charge_capacity, remaining_charge_capacity, avg_power,
	       int_temp, state_of_health, time_to_full, time_to_empty;

	while (1) {
		printk("Observation: %d\n", observation);
		ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_GAUGE_VOLTAGE);
		if (ret < 0) {
			printk("Unable to fetch the voltage\n");
			return;
		}

		ret = sensor_channel_get(dev, SENSOR_CHAN_GAUGE_VOLTAGE,
					 &voltage);
		if (ret < 0) {
			printk("Unable to get the Voltage value\n");
			return;
		}

		printk("Voltage: %d.%06dV\n", voltage.val1, voltage.val2);

		ret = sensor_sample_fetch_chan(dev,
					       SENSOR_CHAN_GAUGE_AVG_CURRENT);
		if (ret < 0) {
			printk("Unable to fetch the Average Current\n");
			return;
		}
		ret = sensor_channel_get(dev, SENSOR_CHAN_GAUGE_AVG_CURRENT,
					 &current);
		if (ret < 0) {
			printk("Unable to get the Current Value\n");
			return;
		}

		bq34110_show_values("Avg Current in Amps", current);

		ret = sensor_sample_fetch_chan(dev,
					SENSOR_CHAN_GAUGE_STATE_OF_CHARGE);
		if (ret < 0) {
			printk("Unable to fetch the State of Charge\n");
			return;
		}

		ret = sensor_channel_get(dev,
					 SENSOR_CHAN_GAUGE_STATE_OF_CHARGE,
					 &state_of_charge);
		if (ret < 0) {
			printk("Unable to get the State of Charge\n");
			return;
		}

		printk("State of Charge: %d%%\n", state_of_charge.val1);

		ret = sensor_sample_fetch_chan(dev,
					SENSOR_CHAN_GAUGE_STATE_OF_HEALTH);
		if (ret < 0) {
			printk("Unable to fetch the State of Health\n");
			return;
		}
		ret = sensor_channel_get(dev,
					 SENSOR_CHAN_GAUGE_STATE_OF_HEALTH,
					 &state_of_health);
		if (ret < 0) {
			printk("Unable to get State of Charge\n");
			return;
		}

		ret = sensor_sample_fetch_chan(dev,
					SENSOR_CHAN_GAUGE_AVG_POWER);
		if (ret < 0) {
			printk("Unable to fetch the Avg Power\n");
			return;
		}
		ret = sensor_channel_get(dev, SENSOR_CHAN_GAUGE_AVG_POWER,
					 &avg_power);
		if (ret < 0) {
			printk("Unable to get Avg Power\n");
			return;
		}

		bq34110_show_values("Avg Power in Watt", avg_power);

		ret = sensor_sample_fetch_chan(dev,
					SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY);
		if (ret < 0) {
			printk("Unable to fetch the Full Charge Capacity\n");
			return;
		}
		ret = sensor_channel_get(dev,
			SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY,
			&full_charge_capacity);
		if (ret < 0) {
			printk("Unable to get Full Charge Capacity\n");
			return;
		}

		printk("Full Charge Capacity: %d.%06dAh\n",
		       full_charge_capacity.val1, full_charge_capacity.val2);

		ret = sensor_sample_fetch_chan(dev,
				SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY);
		if (ret < 0) {
			printk("Unable to fetch the Remaining Charge Capacity\n");
			return;
		}
		ret = sensor_channel_get(dev,
				SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY,
				&remaining_charge_capacity);
		if (ret < 0) {
			printk("Unable to get Remaining Charge Capacity\n");
			return;
		}

		printk("Remaining Charge Capacity: %d.%06dAh\n",
		       remaining_charge_capacity.val1,
		       remaining_charge_capacity.val2);

		ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_GAUGE_TEMP);
		if (ret < 0) {
			printk("Unable to fetch the Temperature\n");
			return;
		}

		ret = sensor_channel_get(dev, SENSOR_CHAN_GAUGE_TEMP,
					 &int_temp);
		if (ret < 0) {
			printk("Unable to read Internal Temperature\n");
			return;
		}

		printk("Gauge Temperature: %d.%06d C\n", int_temp.val1,
		       int_temp.val2);

		ret = sensor_sample_fetch_chan(dev,
					SENSOR_CHAN_GAUGE_TIME_TO_FULL);
		if (ret < 0) {
			printk("Unable to fetch Time to Full\n");
			return;
		}

		ret = sensor_channel_get(dev,
					SENSOR_CHAN_GAUGE_TIME_TO_FULL,
					&time_to_full);
		if (ret < 0) {
			printk("Unable to read Time to Full\n");
			return;
		}

		printk("Time to Full: %dminutes\n", time_to_full.val1);

		ret = sensor_sample_fetch_chan(dev,
					SENSOR_CHAN_GAUGE_TIME_TO_EMPTY);
		if (ret < 0) {
			printk("Unable to fetch Time to Empty\n");
			return;
		}

		ret = sensor_channel_get(dev,
					SENSOR_CHAN_GAUGE_TIME_TO_EMPTY,
					&time_to_empty);
		if (ret < 0) {
			printk("Unable to read Time to Empty\n");
			return;
		}

		printk("Time to empty: %dminutes\n", time_to_empty.val1);

		k_sleep(K_MSEC(20000));

		observation++;
	}
}

void main(void)
{
	const struct device *dev;

	dev = device_get_binding(DT_LABEL(DT_INST(0, ti_bq34110)));
	if (!dev) {
		printk("Failed to get device binding\n");
		return;
	}

	printk("device is %p, name is %s\n", dev, dev->name);

	do_main(dev);
}
