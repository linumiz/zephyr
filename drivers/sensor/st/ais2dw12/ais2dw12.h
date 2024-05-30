/* ST Microelectronics AIS2DW12 3-axis accelerometer driver
 *
 * Copyright (c) 2024 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/ais2dw12.pdf
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AIS2DW12_AIS2DW12_H_
#define ZEPHYR_DRIVERS_SENSOR_AIS2DW12_AIS2DW12_H_

#include <zephyr/types.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <stmemsc.h>
#include "ais2dw12_reg.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <zephyr/drivers/spi.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <zephyr/drivers/i2c.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

enum ais2dw12_mode {
	SINGLE_SHOT_MODE,
	FIFO_MODE,
	STREAM_MODE
};

enum aisdw12_trigger {
	AIS2DW12_DRDY_TRIGGER,		/* Accel Data Ready */
	AIS2DW12_FIFO_THRES_TRIGGER,	/* FIFO Threshold */
	AIS2DW12_FIFO_FULL_TRIGGER,	/* FIFO Full */
	AIS2DW12_FIFO_OVR_TRIGGER,	/* FIFO Overflow */
	AIS2DW12_TEMP_TRIGGER,		/* Temerature Data Ready */
	AIS2DW12_MAX_TRIGGER
}

struct ais2dw12_config {
	stmdev_ctx_t ctx;
	union {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
		const struct i2c_dt_spec i2c;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
		const struct spi_dt_spec spi;
#endif
	} stmemsc_cfg;

#ifdef CONFIG_AIS2DW12_TRIGGER
	const struct gpio_dt_spec int_gpio;
	uint8_t int_cfg;
#endif
	enum ais2dw12_mode mode;
	uint8_t range;
	uint8_t accel_resolution;
};

struct ais2dw12_data {
	int16_t accel_xyx[3];
	uint8_t sensitivity_scale;
	struct k_sem lock;

#ifdef CONFIG_AIS2DW12_ENABLE_TEMP
	int temperature;
#endif

#ifdef CONFIG_AIS2DW12_TRIGGER
	struct gpio_dt_spec *int_gpio;
	struct gpio_callback gpio_cb;
	const struct sensor_trigger *trigger[AIS2DW12_MAX_TRIGGER];
	sensor_trigger_handler_t handler[AIS2DW12_MAX_TRIGGER];
	const struct device *dev;

#if defined(CONFIG_AIS2DW12_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_AIS2DW12_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem trig_sem;
#else
	struct k_work work;
#endif

#endif /* CONFIG_AIS2DW12_TRIGGER */
};

#ifdef CONFIG_AIS2DW12_TRIGGER
int ais2dw12_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);

int ais2dw12_trigger_init(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_AIS2DW12_AIS2DW12_H_ */
