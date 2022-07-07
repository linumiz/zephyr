/* HTU21D temperature & humidity sensor with i2c
 *
 * Copyright (c) 2022 Linumiz
 * Author: Arunmani <arunmani@linumiz.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT te_htu21d

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(htu21d, CONFIG_SENSOR_LOG_LEVEL);

#include "htu21d.h"

#define HTU21D_MODIFIER		((float)(0xFFFF))
#define HTU21D_TEMP_MULTIPLIER	175.72
#define HTU21D_TEMP_OFFSET	-46.85
#define HTU21D_HUM_MULTIPILER	125.0
#define HTU21D_HUM_ADD	-6.0
#define HTU21D_DECIMAL		10000
#define HTU21D_MICRO	100

struct htu21d_data {
	int32_t temp;
	int32_t humidity;
};

struct htu21d_dev_config {
	struct i2c_dt_spec bus;
};

static int send_device_cmd(const struct device *dev, uint16_t *val, uint8_t cmd)
{
	int ret;
	const struct htu21d_dev_config *cfg = dev->config;
	
	/*to write register address*/
	ret = i2c_write_dt(&cfg->bus, &cmd, sizeof(cmd));
	if (ret != 0) {
		return ret;
	}

	/*read values*/
	ret = i2c_read_dt(&cfg->bus, (uint8_t *)val, sizeof(*val));
	if (ret != 0) {
		return ret;
	}

	if (IS_ENABLED(CONFIG_HTU21D_NH_MASTER)) {
		k_sleep(K_MSEC(MEASURE_WAIT_MS));
	}

	*val = sys_get_be16((uint8_t *)val);

	return 0;
}

static int htu21d_sample_fetch(const struct device *dev,
					enum sensor_channel chan)
{
	struct htu21d_data *drv_data = dev->data;
	uint16_t value;
	int ret;
	drv_data->temp = 0U;
	drv_data->humidity = 0U;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		/* Trigger Temperature Measurement*/
		ret = send_device_cmd(dev, &value,
					(IS_ENABLED(CONFIG_HTU21D_NH_MASTER)
					 ? HTU21D_READ_TEMP_NH :
					 HTU21D_READ_TEMP_H));
		if (ret)
			return ret;

		drv_data->temp = value;
		break;
	case SENSOR_CHAN_HUMIDITY:
		/* Trigger Humidity Measurement*/
		ret = send_device_cmd(dev, &value,
					(IS_ENABLED(CONFIG_HTU21D_NH_MASTER)
					 ? HTU21D_READ_HUMID_NH :
					 HTU21D_READ_HUMID_H));
		if (ret)
			return ret;

		drv_data->humidity = value;
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static int htu21d_channel_get(const struct device *dev,
		enum sensor_channel chan,
		struct sensor_value *val)
{
	struct htu21d_data *drv_data = dev->data;
	double result;
	float float_val;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		double sensor_tmp_val;
		int32_t temp_val;

		temp_val = drv_data->temp;

		sensor_tmp_val = temp_val/HTU21D_MODIFIER;

		/*temperature calc*/
		/* Temp = -46.85 + 175.72 * temp_val/2^16 */
		result = HTU21D_TEMP_OFFSET  +
			(HTU21D_TEMP_MULTIPLIER * sensor_tmp_val);
		val->val1 = (int)result;
		float_val = (result - (int)result)*HTU21D_DECIMAL;
		val->val2 = (int)float_val/HTU21D_MICRO;
		break;
	case SENSOR_CHAN_HUMIDITY:
		int32_t humidity_val;
		double sensor_humidity_val;

		humidity_val = drv_data->humidity;
		sensor_humidity_val = humidity_val/HTU21D_MODIFIER;

		/*humidity calc*/
		/* RH = -6 + 125 * humidity_val/2^16 */
		result = HTU21D_HUM_ADD +
			(HTU21D_HUM_MULTIPILER *
			 sensor_humidity_val);
		val->val1 = (int)result;
		float_val = (result-(int)result)*HTU21D_DECIMAL;
		val->val2 = (int)float_val/HTU21D_MICRO;
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static const struct sensor_driver_api htu21d_driver_api = {
	.sample_fetch = htu21d_sample_fetch,
	.channel_get = htu21d_channel_get,
};

static int htu21d_init(const struct device *dev)
{
	const struct htu21d_dev_config *cfg = dev->config;
	
	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C dev %s not ready", cfg->bus.bus->name);
		return -EINVAL;
	}

	/*HTU21D reset*/
	i2c_write_dt(&cfg->bus, (uint8_t *)HTU21D_SOFT_RESET, sizeof(HTU21D_SOFT_RESET));

	return 0;
}

#define HTU21D_DEFINE(inst)						\
	static struct htu21d_data htu21d_drv_data_##inst;		\
	static const struct htu21d_dev_config htu21d_config_##inst = {	\
			.bus = I2C_DT_SPEC_INST_GET(inst)		\
	};								\
	DEVICE_DT_INST_DEFINE(inst,					\
			htu21d_init,					\
			NULL,						\
			&htu21d_drv_data_##inst,			\
			&htu21d_config_##inst,				\
			POST_KERNEL,					\
			CONFIG_SENSOR_INIT_PRIORITY,			\
			&htu21d_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HTU21D_DEFINE)
	
