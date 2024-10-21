/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Linumiz
 */

#include <math.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/cs5490.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "cs5490.h"

LOG_MODULE_REGISTER(CS5490, CONFIG_SENSOR_LOG_LEVEL);

/* Macro's for Parameters Calculations */
#define CS5490_MSBNULL		0
#define CS5490_MSBSIGNED	1
#define CS5490_MSBUNSIGNED	2

static void uart_read_callback(const struct device *dev,
			       struct uart_event *evt, void *user_data)
{
	struct cs5490_data *data = (struct cs5490_data *)user_data;

	switch(evt->type) {
	case UART_TX_DONE:
		k_sem_give(&data->uart_tx_sem);
		printk("TX done\n");
		break;
	case UART_RX_RDY:
		uint8_t *buf = evt->data.rx.buf;

#if 1
		printk("RX len %u\n", evt->data.rx.len);
		for (int i = 0; i < 3; i++)
			printk("0x%x ", buf[i]);
#endif
		if (evt->data.rx.len == 3) {
			k_sem_give(&data->uart_rx_sem);
		}

		break;
	default:
		break;
	}
}

static inline int cs5490_configure_uart(const struct device *dev,
					const struct uart_config *uart_cfg)
{
	return uart_configure(dev, uart_cfg);
}

static int cs5490_get_baudrate(const struct device *dev)
{
	int rc;
	uint32_t baud = 0;
	struct cs5490_rx_frame buf = {0};

	rc = cs5490_select_page(dev, CS5490_PAGE_0);
	if (rc < 0) {
		return rc;
	}

	rc = cs5490_read(dev, REG_SERIAL_CTRL, (uint8_t *)&buf,
			 sizeof(struct cs5490_rx_frame));
	if (rc < 0) {
		return rc;
	}

	baud = sys_get_le24((uint8_t *)&buf);
	baud = baud - 0x020000;
	baud = baud / 0.5242880;
	baud = baud * CS5490_MCLK;

	LOG_INF("baudrate = %u", baud);

	return rc;
}

static int cs5490_set_baudrate(const struct device *dev, uint32_t baudrate)
{
	int rc;
	uint8_t select_page = 0;
	uint32_t hexbaud = 0;
	const struct cs5490_config *cfg = dev->config;
	const struct uart_config uart_cfg = {
		.baudrate = baudrate,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
	};


	select_page = PAGE_SELECT_CMD | CS5490_PAGE_0;
	rc = cs5490_select_page(dev, CS5490_PAGE_0);
	if (rc < 0) {
		return rc;
	}

	hexbaud = ceil(baudrate * 0.5242880 / CS5490_MCLK);
	if (hexbaud > 65535) {
		hexbaud = 65535;
	}

	hexbaud += 0x020000;
	rc = cs5490_write(dev, REG_SERIAL_CTRL, hexbaud);
	if (rc < 0) {
		LOG_ERR("Failed to set baudrate %d", rc);
		return rc;
	}

	/* Wait before change buadrate of uart for flush data*/
	k_msleep(50);

	rc = cs5490_configure_uart(cfg->dev, &uart_cfg);
	if (rc < 0) {
		LOG_ERR("Failed to configure uart %d", rc);
		return rc;
	}

	rc = cs5490_get_baudrate(dev);
	if (rc < 0) {
		LOG_ERR("Failed to get baud rate %d", rc);
		return rc;
	}

	return rc;
}

static float cs5490_convert_to_value(int lsb, int msboption, uint32_t buf)
{
	double output = 0.0;
	bool msb;

	switch(msboption){
	case CS5490_MSBNULL:
		buf &= 0x7FFFFF;
		output = (double)buf;
		output /= pow(2, lsb);
		break;
	case CS5490_MSBSIGNED:
		msb = FIELD_GET(BIT(23), buf);
		if(msb){
			buf = ~buf;
			buf = buf & 0x00FFFFFF;
			output = (double)buf + 1.0;
			output /= -pow(2, lsb);
		} else {
			output = (double)buf;
			output /= (pow(2, lsb)-1.0);
		}
		break;
	default:
	case CS5490_MSBUNSIGNED:
		output = (double)buf;
		output /= pow(2, lsb);
		break;
	}

	return output;
}

static int cs5490_set_overcurrent_threshold(const struct device *dev,
					    uint16_t threshold)
{
	int rc;

	rc = cs5490_select_page(dev, CS5490_PAGE_17);
	if (rc < 0) {
		return rc;
	}

	return cs5490_write(dev, REG_IOVR_LEVEL, threshold);
}

static int cs5490_set_overcurrent_duration(const struct device *dev,
					   uint16_t threshold)
{
	int rc;

	rc = cs5490_select_page(dev, CS5490_PAGE_17);
	if (rc < 0) {
		return rc;
	}

	return cs5490_write(dev, REG_IOVR_DUR, threshold);
}

static int cs5490_attr_set(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	int rc;

	switch ((enum sensor_attribute_cs5490) attr) {
	case SENSOR_ATTR_BAUDRATE:
		rc = cs5490_set_baudrate(dev, val->val1);
		break;
	case SENSOR_ATTR_OVERCURRENT_THRESHOLD:
		rc = cs5490_set_overcurrent_threshold(dev, val->val1);
		break;
	case SENSOR_ATTR_OVERCURRENT_DURATION:
		rc = cs5490_set_overcurrent_duration(dev, val->val1);
		break;
	default:
		rc = -ENOTSUP;
		break;
	}

	return rc;
}

static inline int cs5490_param_fetch(const struct device *dev, uint8_t addr,
				     int8_t lsb, int8_t msb, float *result)
{
	int rc;
	uint32_t val = 0;
	struct cs5490_rx_frame buf = {0};

	rc = cs5490_read(dev, addr, (uint8_t *)&buf,
			 sizeof(struct cs5490_rx_frame));
	if (rc < 0) {
		return rc;
	}

	val = sys_get_le24((uint8_t *)&buf);
	*result = cs5490_convert_to_value(lsb, msb, val);

	return 0;
}

static int cs5490_fetch(const struct device *dev, struct cs5490 *out)
{
	int rc;

	rc = cs5490_select_page(dev, CS5490_PAGE_16);
	if (rc < 0) {
		return rc;
	}

	rc = cs5490_param_fetch(dev, REG_INST_VOLTAGE, 23, CS5490_MSBSIGNED,
				&out->inst_voltage);
	if (rc < 0) {
		return rc;
	}

	printk("volatge %f\n", out->inst_voltage);
	rc = cs5490_param_fetch(dev, REG_INST_POWER, 23, CS5490_MSBSIGNED,
				&out->inst_active_power);
	if (rc < 0) {
		return rc;
	}

	printk("power %f\n", out->inst_active_power);
	rc = cs5490_param_fetch(dev, REG_ACTIVE_POWER, 23, CS5490_MSBSIGNED,
				&out->active_power);
	if (rc < 0) {
		return rc;
	}

	printk("active power %f\n", out->active_power);
	rc = cs5490_param_fetch(dev, REG_RMS_CURRENT, 24, CS5490_MSBUNSIGNED,
				&out->rms_current);
	if (rc < 0) {
		return rc;
	}

	printk("rms current %f\n", out->rms_voltage);
	rc = cs5490_param_fetch(dev, REG_RMS_VOLTAGE, 24, CS5490_MSBUNSIGNED,
				&out->rms_voltage);
	if (rc < 0) {
		return rc;
	}

	printk("rms volatge %f\n", out->rms_voltage);
	rc = cs5490_param_fetch(dev, REG_REACT_POWER, 23, CS5490_MSBSIGNED,
				&out->reactive_power);
	if (rc < 0) {
		return rc;
	}
	printk("reactive power %f\n", out->reactive_power);

	rc = cs5490_param_fetch(dev, REG_APPARENT_POWER, 23, CS5490_MSBSIGNED,
				&out->apparent_power);
	if (rc < 0) {
		return rc;
	}
	printk("apprent power %f\n", out->apparent_power);

	rc = cs5490_param_fetch(dev, REG_POWER_FACTOR, 23, CS5490_MSBSIGNED,
				&out->power_factor);
	if (rc < 0) {
		return rc;
	}

	printk("power factor %f\n", out->power_factor);
	rc = cs5490_select_page(dev, CS5490_PAGE_0);
	if (rc < 0) {
		return rc;
	}

	rc = cs5490_param_fetch(dev, REG_PEAK_CURRENT, 23, CS5490_MSBSIGNED,
				&out->peak_current);
	if (rc < 0) {
		return rc;
	}
	printk("peak current %f\n", out->peak_current);
	rc = cs5490_param_fetch(dev, REG_PEAK_VOLTAGE, 23, CS5490_MSBSIGNED,
				&out->peak_voltage);
	if (rc < 0) {
		return rc;
	}
	printk("peak voltage %f\n", out->peak_voltage);

	return 0;
}

static int cs5490_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct cs5490_data *data = dev->data;

	return cs5490_fetch(dev, &data->sensor_data);
}

static int cs5490_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	int rc;
	struct cs5490_data *data = dev->data;
	struct cs5490 *rdata = &data->sensor_data;

	switch ((enum sensor_channel_cs5490)chan) {
	case SENSOR_CHAN_INST_CURRENT:
		rc = sensor_value_from_double(val, rdata->inst_current);
		break;
	case SENSOR_CHAN_INST_VOLTAGE:
		rc = sensor_value_from_double(val, rdata->inst_voltage);
		break;
	case SENSOR_CHAN_INST_POWER:
		rc = sensor_value_from_double(val, rdata->inst_active_power);
		break;
	case SENSOR_CHAN_RMS_CURRNT:
		rc = sensor_value_from_double(val, rdata->rms_current);
		break;
	case SENSOR_CHAN_RMS_VOLTAGE:
		rc = sensor_value_from_double(val, rdata->rms_voltage);
		break;
	case SENSOR_CHAN_ACTIVE_POWER:
		rc = sensor_value_from_double(val, rdata->active_power);
		break;
	case SENSOR_CHAN_APPARENT_POWER:
		rc = sensor_value_from_double(val, rdata->apparent_power);
		break;
	case SENSOR_CHAN_REACTIVE_POWER:
		rc = sensor_value_from_double(val, rdata->reactive_power);
		break;
	case SENSOR_CHAN_POWER_FACTOR:
		rc = sensor_value_from_double(val, rdata->power_factor);
		break;
	case SENSOR_CHAN_PEAK_CURRENT:
		rc = sensor_value_from_double(val, rdata->peak_current);
		break;
	case SENSOR_CHAN_PEAK_VOLTAGE:
		rc = sensor_value_from_double(val, rdata->peak_voltage);
		break;
	default:
		rc = -ENOTSUP;
		break;
	}

	return rc;
}

#if defined(CONFIG_CS5490_TRIGGER)
static inline int cs5490_trigger_set(const struct device *dev,
			      const struct sensor_trigger *trig,
			      sensor_trigger_handler_t handler)
{
	int rc;

	rc = cs5490_configure_trigger(dev, trig, handler);
	if (rc < 0) {
		LOG_ERR("Failed to set trigger (%d)", rc);
	}

	return rc;
}
#endif

static int cs5490_init(const struct device *dev)
{
	int rc;
	struct cs5490_data *data = dev->data;
	const struct cs5490_config *cfg = dev->config;
	const struct uart_config uart_cfg = {
		.baudrate = 600,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
	};

	if (!device_is_ready(cfg->dev)) {
		LOG_ERR("Error: CS5490 is not ready");
		return -ENODEV;
	}

	rc = cs5490_configure_uart(cfg->dev, &uart_cfg);
	if (rc < 0) {
		LOG_ERR("Failed to configure uart %d", rc);
		return rc;
	}

	uart_rx_disable(cfg->dev);
	rc = uart_callback_set(cfg->dev, uart_read_callback, data);
	if (rc < 0) {
		LOG_ERR("Failed to set sensor callback %d", rc);
		return rc;
	}

	k_sem_init(&data->lock, 1, 1);
	k_sem_init(&data->uart_tx_sem, 0, 1);
	k_sem_init(&data->uart_rx_sem, 0, 1);

	rc = cs5490_get_baudrate(dev);
	if (rc < 0) {
		LOG_ERR("Failed to get baudrate %d", rc);
		return rc;
	}
#if defined(CONFIG_CS5490_TRIGGER)
	rc = cs5490_trigger_init(dev);
	if (rc < 0) {
		LOG_ERR("Failed to initialize interrupts");
		return rc;
	}
#endif /* CONFIG_CS5490_TRIGGER */

	LOG_INF("CS5490 Initialized");

	return 0;
}

static const struct sensor_driver_api cs5490_api = {
	.attr_set	= cs5490_attr_set,
#if defined(CONFIG_CS5490_TRIGGER)
	.trigger_set	= cs5490_trigger_set,
#endif
	.sample_fetch	= cs5490_sample_fetch,
	.channel_get	= cs5490_channel_get,
};

#define CS5490_DEFINE(idx)                                                           \
                                                                                     \
	static const struct cs5490_config cs5490_config_##idx = {                    \
		.dev = DEVICE_DT_GET(DT_INST_BUS(idx)),                              \
		IF_ENABLED(CONFIG_CS5490_TRIGGER,				     \
		(.int_gpio = GPIO_DT_SPEC_INST_GET_OR(idx, int_gpios, {}),)) };	     \
	static struct cs5490_data cs5490_data_##idx;                                 \
	DEVICE_DT_INST_DEFINE(idx, cs5490_init, NULL, &cs5490_data_##idx,     	     \
				     &cs5490_config_##idx, POST_KERNEL,              \
				     CONFIG_SENSOR_INIT_PRIORITY, &cs5490_api);

#define DT_DRV_COMPAT cirrus_cs5490
DT_INST_FOREACH_STATUS_OKAY(CS5490_DEFINE)
