#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/cs5490.h>

#define SLEEP_TIME	K_MSEC(1000)

static int fetch_and_display(const struct device *dev)
{
	int rc;
	struct sensor_value rms_current;
	struct sensor_value rms_voltage;
	struct sensor_value power;
	struct sensor_value inst_voltage;
	struct sensor_value freq;

	rc = sensor_sample_fetch(dev);
	if (rc < 0) {
		printk("Failed to fetch sample %d\n", rc);
		return rc;
	}

#if 0
	rc = sensor_channel_get(dev, SENSOR_CHAN_INST_VOLTAGE, &inst_voltage); 
	if (rc < 0) {
		printk("Failed to get sample %d\n", rc);
		return rc;
	}

	rc = sensor_channel_get(dev, SENSOR_CHAN_RMS_CURRNT, &rms_current); 
	if (rc < 0) {
		printk("Failed to get sample %d\n", rc);
		return rc;
	}

	rc = sensor_channel_get(dev, SENSOR_CHAN_RMS_VOLTAGE, &rms_voltage); 
	if (rc < 0) {
		printk("Failed to get sample %d\n", rc);
		return rc;
	}
	rc = sensor_channel_get(dev, SENSOR_CHAN_FREQUENCY, &freq);
	if (rc < 0) {
		printk("Failed to get frequency %d\n", rc);
		return rc;
	}

	printk("Inst voltage %f\n", sensor_value_to_float(&inst_voltage));
	printk("Frequency %f\n", sensor_value_to_float(&freq));

#endif
	return 0;
}

#if defined CONFIG_CS5490_TRIGGER
static void trigger_handler(const struct device *dev,
			    const struct sensor_trigger *trig)
{
	switch ((enum sensor_trigger_type_cs5490)trig->type) {
	case SENSOR_TRIG_OVERCURRENT:
		printk("Over current is detected\n");
		break;
	case SENSOR_TRIG_OUT_OF_RANGE_VOLTAGE:
		printk("Voltage out of Range\n");
		break;
	case SENSOR_TRIG_OUT_OF_RANGE_CURRENT:
		printk("Current out of Range\n");
		break;
	case SENSOR_TRIG_OUT_OF_RANGE_POWER:
		printk("Current out of Range\n");
		break;
	case SENSOR_TRIG_DATA_READY:
	default:
		fetch_and_display(dev);
		break;
	}
}
#endif

int main(void)
{
	int rc;
	struct sensor_value baudrate = {0};
	const struct device *const sensor = DEVICE_DT_GET_ONE(cirrus_cs5490);

	if (!device_is_ready(sensor)) {
		printk("Device is not ready \n");
		return 0;
	}

#if 0
	/* Setting baudrate to 115200 */
	baudrate.val1 = 115200;
	rc = sensor_attr_set(sensor, 1,
			     SENSOR_ATTR_BAUDRATE,
			     &baudrate);
	if (rc != 0) {
		printk("Failed to set baudrate %d\n", rc);
		return 0;
	}
#endif
#if defined CONFIG_CS5490_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	rc = sensor_trigger_set(sensor, &trig, trigger_handler);
	if (rc != 0) {
		printk("Failed to set trigger: %d\n", rc);
		return 0;
	}

#if 0
	trig.type = SENSOR_TRIG_OUT_OF_RANGE_VOLTAGE;
	rc = sensor_trigger_set(sensor, &trig, trigger_handler);
	if (rc != 0) {
		printk("Failed to set trigger: %d\n", rc);
		return 0;
	}

	trig.type = SENSOR_TRIG_OUT_OF_RANGE_CURRENT;
	rc = sensor_trigger_set(sensor, &trig, trigger_handler);
	if (rc != 0) {
		printk("Failed to set trigger: %d\n", rc);
		return 0;
	}

	trig.type = SENSOR_TRIG_OUT_OF_RANGE_POWER;
	rc = sensor_trigger_set(sensor, &trig, trigger_handler);
	if (rc != 0) {
		printk("Failed to set trigger: %d\n", rc);
		return 0;
	}

	struct sensor_value attr_value = {0};
	attr_value.val1 = 10;
	rc = sensor_attr_set(sensor, 1,
			     SENSOR_ATTR_OVERCURRENT_THRESHOLD,
			     &attr_value);
	if (rc != 0) {
		printf("Failed to set overcurrent threshold %d\n", rc);
		return 0;
	}

	trig.type = SENSOR_TRIG_OVERCURRENT;
	rc = sensor_trigger_set(sensor, &trig, trigger_handler);
	if (rc != 0) {
		printk("Failed to set trigger: %d\n", rc);
		return 0;
	}
#else
#endif
#endif
	/* Polling Mode */
	while (1) {
		fetch_and_display(sensor);

		k_sleep(K_SECONDS(1));
	}

	return 0;
}
