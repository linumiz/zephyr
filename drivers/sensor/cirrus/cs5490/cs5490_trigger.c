/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Linumiz
 */

#include "cs5490.h"
#include <zephyr/drivers/sensor/cs5490.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(CS5490, CONFIG_SENSOR_LOG_LEVEL);

static void cs5490_gpio_callback(const struct device *dev,
				 struct gpio_callback *cb,
				 uint32_t pin_mask)
{
	struct cs5490_data *data = CONTAINER_OF(cb, struct cs5490_data, int_gpio_cb);

	const struct cs5490_config *cfg = data->int_gpio->config;

	if ((pin_mask & (BIT(cfg->int_gpio.pin))) == 0U) {
		return;
	}

	gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);

#if defined(CONFIG_CS5490_TRIGGER_OWN_THREAD)
	k_sem_give(&data->trig_sem);
#else
	k_work_submit(&data->work);
#endif
}

static void cs5490_process_interrupt(const struct device *dev)
{
	int rc;
	uint32_t int_source = 0;
	struct cs5490_rx_frame buf = {0};
	const struct cs5490_config *cfg = dev->config;
	struct cs5490_data *data = dev->data;

	rc = cs5490_select_page(dev, CS5490_PAGE_0);
	if (rc < 0) {
		return;
	}

	rc = cs5490_read(dev, REG_INT_STATUS, (uint8_t *)&buf, sizeof(buf));
	if (rc < 0) {
		return;
	}

	int_source = sys_get_le24((uint8_t*)&buf);
	if (int_source & CS5490_INT_DRDY_MASK) {
		if (data->handler[CS5490_INT_DRDY]) {
			data->handler[CS5490_INT_DRDY](dev, data->trigger[CS5490_INT_DRDY]);
		}
	}

	if (int_source & CS5490_INT_POR_MASK) {
		if (data->handler[CS5490_INT_POR]) {
			data->handler[CS5490_INT_POR](dev, data->trigger[CS5490_INT_POR]);
		}
	}

	if (int_source & CS5490_INT_IOR_MASK) {
		if (data->handler[CS5490_INT_IOR]) {
			data->handler[CS5490_INT_IOR](dev, data->trigger[CS5490_INT_IOR]);
		}
	}

	if (int_source & CS5490_INT_VOR_MASK) {
		if (data->handler[CS5490_INT_VOR]) {
			data->handler[CS5490_INT_VOR](dev, data->trigger[CS5490_INT_VOR]);
		}
	}

	if (int_source & CS5490_INT_IOR_MASK) {
		if (data->handler[CS5490_INT_IOR]) {
			data->handler[CS5490_INT_IOC](dev, data->trigger[CS5490_INT_IOC]);
		}
	}

	if (int_source & CS5490_INT_VSAG_MASK) {
		if (data->handler[CS5490_INT_VSAG]) {
			data->handler[CS5490_INT_VSAG](dev, data->trigger[CS5490_INT_VSAG]);
		}
	}

	if (int_source & CS5490_INT_VSWELL_MASK) {
		if (data->handler[CS5490_INT_VSWELL]) {
			data->handler[CS5490_INT_VSWELL](dev, data->trigger[CS5490_INT_VSWELL]);
		}
	}

	rc = cs5490_write(dev, REG_INT_STATUS, int_source);
	if (rc < 0) {
		LOG_ERR("Failed to clear interrupts");
		return;
	}

	gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_FALLING);

	return;
}

#if defined(CONFIG_CS5490_TRIGGER_OWN_THREAD)
static void cs5490_thread(struct cs5490_data *data)
{
	while(true) {
		k_sem_take(&data->trig_sem, K_FOREVER);
		cs5490_process_int(data->int_gpio);
	}
}
#else
static void cs5490_work_cb(struct k_work *work)
{
	struct cs5490_data *data = CONTAINER_OF(work, struct cs5490_data, work);

	cs5490_process_interrupt(data->int_gpio);
}
#endif

int cs5490_configure_trigger(const struct device *dev,
			     const struct sensor_trigger *trig,
			     sensor_trigger_handler_t handler)
{
	int rc;
	uint32_t mask = 0;
	const struct cs5490_config *cfg = dev->config;
	struct cs5490_rx_frame buf = {0};
	struct cs5490_data *data = dev->data;
	enum sensor_trigger_type_cs5490 trig_type = (enum sensor_trigger_type_cs5490) trig->type;

	if (!((enum sensor_trigger_type) trig_type & SENSOR_TRIG_DATA_READY) &&
	    !(trig_type & SENSOR_TRIG_OVERCURRENT) &&
	    !(trig_type & SENSOR_TRIG_OUT_OF_RANGE_VOLTAGE) &&
	    !(trig_type & SENSOR_TRIG_OUT_OF_RANGE_CURRENT) &&
	    !(trig_type & SENSOR_TRIG_OUT_OF_RANGE_POWER)) {
		LOG_ERR("Unsupported sensor trigger");
		return -ENOTSUP;
	}

	rc = cs5490_select_page(dev, CS5490_PAGE_0);
	if (rc < 0) {
		return rc;
	}

	mask = CS5490_INT_CFG_MASK;
	rc = cs5490_write(dev, REG_CONFIG_1, mask);
	if (rc < 0) {
		return rc;
	}

	rc = cs5490_read(dev, REG_INT_MASK, (uint8_t *)&buf, sizeof(buf));
	if (rc < 0) {
		return rc;
	}

	mask = sys_get_le24((uint8_t*)&buf);
	if ((enum sensor_trigger_type) trig_type == SENSOR_TRIG_DATA_READY) {
		data->handler[CS5490_INT_DRDY] = handler;
		data->trigger[CS5490_INT_DRDY] = trig;
		mask |= CS5490_INT_DRDY_MASK;
	}

	if (trig_type == SENSOR_TRIG_OVERCURRENT) {
		data->handler[CS5490_INT_IOC] = handler;
		data->trigger[CS5490_INT_IOC] = trig;
		mask |= CS5490_INT_IOC_MASK;
	}

	if (trig_type == SENSOR_TRIG_OUT_OF_RANGE_VOLTAGE) {
		data->handler[CS5490_INT_VOR] = handler;
		data->trigger[CS5490_INT_VOR] = trig;
		mask |= CS5490_INT_VOR_MASK;
	}

	if (trig_type == SENSOR_TRIG_OUT_OF_RANGE_CURRENT) {
		data->handler[CS5490_INT_IOR] = handler;
		data->trigger[CS5490_INT_IOR] = trig;
		mask |= CS5490_INT_IOR_MASK;
	}

	if (trig_type == SENSOR_TRIG_OUT_OF_RANGE_POWER) {
		data->handler[CS5490_INT_POR] = handler;
		data->trigger[CS5490_INT_POR] = trig;
		mask |= CS5490_INT_POR_MASK;
	}

	rc = cs5490_write(dev, REG_INT_MASK, mask);
	if (rc < 0) {
		return rc;
	}

	rc = cs5490_read(dev, REG_INT_MASK, (uint8_t *)&buf, sizeof(buf));
	if (rc < 0) {
		return rc;
	}

	gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_FALLING);

	return 0;
}

int cs5490_trigger_init(const struct device *dev)
{
	int rc;
	struct cs5490_data *data = dev->data;
	const struct cs5490_config *cfg = dev->config;

	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		LOG_ERR("GPIO Port %s is not ready", cfg->int_gpio.port->name);
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (rc < 0) {
		LOG_ERR("Failed configure interrupt gpio %d", rc);
		return rc;
	}

	data->int_gpio = dev;
#if defined(CONFIG_CS5490_TRIGGER_OWN_THREAD)
	k_sem_init(&data->trig_sem, 0, 1);
	k_thread_create(&data->thread, data->thread_stack,
			CONFIG_CS5490_THREAD_STACK_SIZE,
			(k_thread_entry_t)cs5490_thread, data, NULL,
			NULL, K_PRIO_COOP(CONFIG_CS5490_THREAD_PRIORITY),
			0, K_NO_WAIT);
#else
	k_work_init(&data->work, cs5490_work_cb);
#endif
	gpio_init_callback(&data->int_gpio_cb, cs5490_gpio_callback,
			   BIT(cfg->int_gpio.pin));

	rc = gpio_add_callback(cfg->int_gpio.port, &data->int_gpio_cb);
	if (rc < 0) {
		LOG_ERR("Failed to set interrupt callback");
		return rc;
	}

	rc = cs5490_write(dev, REG_CONFIG_0, 0x00000F);
	if (rc < 0) {
		LOG_ERR("Failed to configure DO pin as interrupt %d", rc);
		return rc;
	}


	return rc;
}
