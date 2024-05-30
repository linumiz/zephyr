/* ST Microelectronics AIS2DW12 3-axis accelerometer driver
 *
 * Copyright (c) 2024 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/ais2dw12.pdf
 */

#define DRV_COMPACT st_ais2dw12

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/sensor/ais2dw12.h>
#include "ais1dw12.h"

LOG_MODULE_REGISTER(AIS2DW12, CONFIG_SENSOR_LOG_LEVEL);

#define AIS2DW12_BDU_SET	1
#define AIS2DW12_OP_MASK	3
#define AIS2DW12_DT_ODR_MAX	6
#define AIS2DW12_RANGE_MAX	2
#define AIS2DW12_SENSE_RANGE	4
#define AIS2DW12_12BIT_2G	0
#define AIS2DW12_12BIT_4G	1
#define AIS2DW12_14BIT_2G	2
#define AIS2DW12_14BIT_4G	3

struct sensitivity_map {
	uint8_t range;
	uint8_t resolution;
};

struct sensitivity_map s_map[AIS2DW12_SENSE_RANGE] = {
	{0, 0}, {0, 1},
	{1, 0}, {1, 1}
};

static int ais2dw12_get_sensitivity(uint8_t range, uint8_t resolution)
{
	for(int i = 0; i < AIS2DW12_SENSE_RANGE; i++) {
		if (range == s_map[i].range &&
		    resolution == s_map[i].resolution) {
			return i;
		}
	}

	return -EINVAL;
}

static int ais2dw12_to_sensor_value(struct device *dev, int16_t *raw_data,
				    struct sensor_value *val)
{
	int rc;
	double value;
	const struct ais2dw12_config *cfg = dev->config;

	rc = ais2dw12_get_sensitivity(cfg->range, cfg->resolution);
	if (rc < 0) {
		return -EINVAL;
	}

	switch (rc) {
	case AIS2DW12_12BIT_2G:
		value = ais2dw12_from_fs2_12bit_to_mg(raw_data);
		break;
	case AIS2DW12_12BIT_4G:
		value = ais2dw12_from_fs2_to_mg(raw_data);
		break;
	case AIS2DW12_14BIT_2G:
		value = ais2dw12_from_fs4_12bit_to_mg(raw_data);
		break;
	case AIS2DW12_14BIT_4G:
		value = ais2dw12_from_fs4_to_mg(raw_data);
		break;
	default:
		return -EINVAL;
	}

	return sensor_value_from_double(val, value);
}

static int ais2dw12_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	int rc;
	const struct ais2dw12_data *data = dev->data;

	k_sem_take(&data->lock, K_FOREVER);
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		rc = ais2dw12_to_sensor_value(dev, &data->accel_xyz[0], val);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		rc = ais2dw12_to_sensor_value(dev, &data->accel_xyz[1], val);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		rc = ais2dw12_to_sensor_value(dev, &data->accel_xyz[2], val);
		break;
	case SENSOR_CHAN_XYZ:
		rc = ais2dw12_to_sensor_value(dev, &data->accel_xyz[0], val);
		rc |= ais2dw12_to_sensor_value(dev, &data->accel_xyz[1], val);
		rc |= ais2dw12_to_sensor_value(dev, &data->accel_xyz[2], val);
		break;
#if defined CONFIG_AIS2DW12_ENABLE_TEMP
	case SENSOR_CHAN_AMBIENT_TEMP:
		double temp = ais2dw12_from_lsb_to_celsius(&data->temperature);
		rc = sensor_value_from_double(val, temp);
		break;
#endif
	default:
		LOG_ERR("Unsupported accel chan");
		rc = -ENOTSUP;
	}

	k_sem_give(&data->lock);

	return rc;
}

static int ais2dw12_sample_fetch(const struct device *dev,
				  enum sensor_channel chan)
{
	int rc;
	const struct ais2dw12_config *cfg = dev->config;
	const struct ais2dw12_data *data = dev->data;

	k_sem_take(&data->lock, K_FOREVER);
	switch (chan) {
	case SENSOR_CHAN_X:
	case SENSOR_CHAN_Y:
	case SENSOR_CHAN_Z:
	case SENSOR_CHAN_XYZ:
		rc = ais2dw12_acceleration_raw_get(&cfg->ctx,
						   &data->accel_xyz[0]);
		break;
#if CONFIG_AIS2DW12_ENABLE_TEMP
	case SENSOR_CHAN_AMBIENT_TEMP:
		rc = ais2dw12_temperature_raw_get(&cfg->ctx,
						  &data->temperature);
		break;
#endif
	default:
		rc = -ENOTSUP;
		break;
	}

	k_sem_give(&data->lock);

	return rc;
}

static int ais2dw12_attr_set_odr(const struct device *dev,
				 const struct sensor_value *val)
{
	int rc;
	const struct ais2dw12_config *cfg = dev->config;
	const struct ais2dw12_data *data = dev->data;

	if (val->val1 > AIS2DW12_DT_ODR_MAX)
		return -EINVAL;

	k_sem_take(&data->sem, K_FOREVER);
	rc = ais2dw12_data_rate_set(&cfg->ctx, val->val1);
	k_sem_give(&data->sem);

	return rc;
}

static int ais2dw12_attr_set_range(const struct device *dev,
				   const struct sensor_value *val)
{
	int rc;
	const struct ais2dw12_config *cfg = dev->config;
	const struct ais2dw12_data *data = dev->data;

	if (val->val1 > AIS2DW12_RANGE_MAX) {
		return -EINVAL;
	}

	data->sensitivity = val->val1;
	k_sem_take(&data->sem, K_FOREVER);
	rc = ais2dw12_full_scale_set(&cfg->ctx, val->val1);
	k_sem_give(&data->sem);

	return rc;
}

static int ais2dw12_attr_set_op_mode(const struct device *dev,
				     const struct sensor_value *val)
{
	int rc;
	enum ais2dw12_mode mode;
	const struct ais2dw12_config *cfg = dev->config;
	const struct ais2dw12_data *data = dev->data;

	if (val->val1 > AIS2DW12_POWER_MODE_4) {
		WRITE_BIT(val->val1, AIS2DW12_OP_MASK, 1);
		if (val->val1 > AIS2DW12_SINGLE_PWR_MD_4) {
			return -EINVAL;
		}

		mode = STREAM_MODE;
	} else {
		if (val->val1 > AIS2DW12_PWR_MD_4)
			return -EINVAL;

		mode = SINGLE_SHOT_MODE;
	}

	k_sem_take(&data->lock, K_FOREVER);
	rc = ais2dw12_power_mode_set(&cfg->ctx, val->val1);
	if (!rc)
		cfg->mode = mode;

	k_sem_give(&data->lock);

	return rc;
}

static int ais2dw12_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_ACCEL_X &&
	    chan != SENSOR_CHAN_ACCEL_Y &&
	    chan != SENSOR_CHAN_ACCEL_Z &&
	    chan != SENSOR_CHAN_ACCEL_XYZ) {
		LOG_ERR("Not supported on this channel.");
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return ais2dw12_attr_set_odr(dev, val);
	case SENSOR_ATTR_FULL_SCALE:
		return ais2dw12_attr_set_range(dev, val);
	case SENSOR_ATTR_CONFIGURATION:
		return ais2dw12_attr_set_op_mode(dev, val);
	default:
		LOG_ERR("Unsupported accel attribute");
		return -ENOTSUP;
	}
}

#if defined(CONFIG_AIS2DW12_TRIGGER)
static int ais2dw12_trigger_set(const struct device *dev,
				const struct sensor_trigger *trigger,
				sensor_trigger_handler_t handler)
{
	int rc;
	const struct ais2dw12_data *data = dev->data;

	k_sem_take(&data->lock, K_FOREVER);
	rc = ais2dw12_configure_trigger(dev, trigger, handler);
	k_sem_give(&data->lock);

	return rc;
}
#endif

static const struct sensor_driver_api ais2dw12_driver_api = {
	.attr_set = ais2dw12_attr_set,
#if defined(CONFIG_AIS2DW12_TRIGGER)
	.trigger_set = ais2dw12_trigger_set,
#endif
	.sample_fetch = ais2dw12_sample_fetch,
	.channel_get = ais2dw12_channel_get,
};

static int ais2dw12_init(const struct device *dev)
{
	int rc;
	uint8_t chip_id = 0;
	const struct ais2dw12_config *cfg = dev->config;
	const struct ais2dw12_data *data = dev->data;
	stmdev_ctx *ctx = &cfg->ctx;

	rc = ais2dw12_device_id_get(ctx, &chip_id);
	if (rc < 0) {
		LOG_ERR("Failed to read %s device (%d)", dev->name);
		return rc;
	}

	if (chip_id != AIS2DW12_ID) {
		LOG_ERR("Invalid chip id");
		return -ENODEV;
	}

	rc = ais2dw12_block_data_update_set(&cfg->ctx, AIS2DW12_BDU_SET);
	if (rc < 0) {
		LOG_WRN("Failed to set BDU");
	}

#if defined(CONFIG_AIS2DW12_TRIGGER)
	rc = ais2dw12_trigger_init(dev);
	if (rc < 0) {
		LOG_ERR("Failed to configure interrupt (%d)", rc);
		return rc;
	}
#endif

	k_sem_init(&data->lock, 0, 1);

	return 0;
}

#define AIS2DW12_DEVICE_INIT(inst)						\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ais2dw12_init, NULL,			\
				     &ais2dw12_data_##inst,			\
				     &ais2dw12_config_##inst, POST_KERNEL,	\
				     CONFIG_SENSOR_INIT_PRIORITY,		\
				     &ais2dw12_driver_api);

/*
 * Instantiation macros used when a device is on a SPI bus.
 */

#ifdef CONFIG_AIS2DW12_TRIGGER
#define AIS2DW12_CFG_IRQ(inst)							\
	.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),		\
	.int_cfg = DT_INST_PROP(inst, int_cfg),
#else
#define AIS2DW12_CFG_IRQ(inst)
#endif /* CONFIG_AIS2DW12_TRIGGER */

#define AIS2DW12_SPI_OPERATION							\
	(SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA)

#define AIS2DW12_CONFIG_SPI(inst)						\
	{									\
		STMEMSC_CTX_SPI(&ais2dw12_config_##inst.stmemsc_cfg),		\
		.stmemsc_cfg = {						\
			.spi = SPI_DT_SPEC_INST_GET(inst,			\
						    AIS2DW12_SPI_OPERATION, 0),	\
		},								\
		AIS2DW12_CFG_IRQ(inst)						\
	}

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define AIS2DW12_CONFIG_I2C(inst)						\
	{									\
		STMEMSC_CTX_I2C(&ais2dw12_config_##inst.stmemsc_cfg),		\
		.stmemsc_cfg = {						\
			.i2c = I2C_DT_SPEC_INST_GET(inst),			\
		},								\
		AIS2DW12_CFG_IRQ(inst)						\
	}

/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define AIS2DW12_DEFINE(inst)								\
	static struct ais2dw12_data ais2dw12_data_##inst;				\
	static const struct ais2dw12_config ais2dw12_config_##inst =			\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi), (AIS2DW12_CONFIG_SPI(inst)),	\
			    (AIS2DW12_CONFIG_I2C(inst)));				\
	AIS2DW12_DEVICE_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(AIS2DW12_DEFINE)
