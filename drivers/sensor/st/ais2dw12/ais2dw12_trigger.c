/* ST Microelectronics AIS2DW12 3-axis accelerometer driver
 *
 * Copyright (c) 2024 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/ais2dw12.pdf
 */

#define DT_DRV_COMPAT st_ais2dw12

#include <zephyr/logging/log.h>
#include "ais2dw12.h"

LOG_MODULE_DECLARE(AIS2DW12, CONFIG_SENSOR_LOG_LEVEL);

static void ais2dw12_gpio_callback(const struct device *dev,
				   struct gpio_callback *cb, uint32_t pins)
{
	int rc;
	struct ais2dw12_data *data = CONTAINER_OF(cb,
						  struct ais2dw12_data, gpio_cb);

	const struct ais2dw12_config *cfg = data->gpio_dev->config;


	if ((pins & BIT(cfg->int_gpio.pin)) == 0U) {
		return;
	}

#if defined(CONFIG_AIS2DW12_TRIGGER_OWN_THREAD)
	k_sem_give(&data->trig_sem);
#else
	k_work_submit(&data->work);
#endif
}

static void ais2dw12_handle_drdy_int(const struct device *dev)
{
	struct ais2dw12_data *data = dev->data;

	if (data->data_ready_handler != NULL) {
		data->data_ready_handler(dev, data->data_ready_trigger);
	}
}

static void ais2dw12_handle_int(const struct device *dev)
{
	struct ais2dw12_data *ais2dw12 = dev->data;
	const struct ais2dw12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	ais2dw12_all_sources_t sources;
	int rc;

	ais2dw12_all_sources_get(ctx, &sources);

}

#ifdef CONFIG_AIS2DW12_TRIGGER_OWN_THREAD
static void ais2dw12_thread(struct ais2dw12_data *data)
{
	while (1) {
		k_sem_take(&data->trig_sem, k_forever);
		ais2dw12_handle_int(data->dev);
	}
}
#endif

#ifdef CONFIG_AIS2DW12_TRIGGER_GLOBAL_THREAD
static void ais2dw12_work_cb(struct k_work *work)
{
	struct ais2dw12_data *data = CONTAINER_OF(work,
						  struct ais2dw12_data, work);

	ais2dw12_handle_int(data->dev);
}
#endif

static int ais2dw12_init_interrupt(const struct device *dev)
{
	int rc;
	const struct ais2dw12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	/* enable latched mode */
	rc = ais2dw12_int_notification_set(&cfg->ctx, AIS2DW12_DRDY_LATCHED);
	if (rc < 0) {
		return rc;
	}

	/* route interrupts to respective pins */
	if (cfg->int_cfg == 1) {
		rc = ais2dw12_all_on_int1_set(&cfg->ctx, 0xFF);
	} else {
		ais2dw12_ctrl5_int2_t cfg_value = (ais2dw12_ctrl5_int2_t)0x1F;
		rc =  ais2dw12_pin_int2_route_set(&cfg->ctx, &cfg_value);
	}

	return rc;
}

int ais2dw12_trigger_init(const struct device *dev)
{
	int rc;
	const struct ais2dw12_data *data = dev->data;
	const struct ais2dw12_config *cfg = dev->config;

	data->int_gpio = (struct gpio_dt_spec *)&cfg->int_gpio;

	if (!gpio_is_ready_dt(data->int_gpio)) {
		LOG_ERR("Cannot get pointer to interrupt gpio device");
		return -ENODEV;
	}

	data->dev = dev;
	rc = gpio_pin_configure_dt(data->int_gpio, GPIO_INPUT);
	if (rc < 0) {
		LOG_ERR("Could not configure interrupt gpio");
		return rc;
	}

	gpio_init_callback(&data->gpio_cb, ais2dw12_gpio_callback,
			   BIT(data->int_gpio->pin));

	rc = gpio_add_callback(data->int_gpio->port, &data->gpio_cb);
	if (rc < 0) {
		LOG_ERR("Could not set gpio callback");
		return rc;
	}

#if defined(CONFIG_AIS2DW12_TRIGGER_OWN_THREAD)
	k_sem_init(&data->trig_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack,
			CONFIG_AIS2DW12_THREAD_STACK_SIZE,
			(k_thread_entry_t)ais2dw12_thread, data, NULL, NULL,
			K_PRIO_COOP(CONFIG_AIS2DW12_THREAD_PRIORITY), 0, K_NO_WAIT);

	k_thread_name_set(&data->thread, dev->name);
#else
	data->work.handler = ais2dw12_work_cb;
#endif

	return gpio_pin_interrupt_configure_dt(data->int_gpio,
					       GPIO_INT_EDGE_TO_ACTIVE);
}

int ais2dw12_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	int rc;
	enum ais2dw12_trigger trigger;
	struct ais2dw12_data *data = dev->data;
	const struct ais2dw12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	if (data->int_gpio->port == NULL) {
		LOG_ERR("trigger_set is not supported");
		return -ENOTSUP;
	}

	rc = gpio_pin_interrupt_configure_dt(data->int_gpio, GPIO_INT_DISABLE);
	if (rc < 0) {
		LOG_ERR("%s: Not able to configure pin_int", dev->name);
		return rc;
	}

	switch (trig.type) {
	case: SENSOR_TRIG_DATA_READY
		data->handler[AIS2DW12_DRDY_TRIGGER] = handler;
		data->trigger[AIS2DW12_DRDY_TRIGGER] = trig;

		/* temp data ready is only routed to interrupt pin 2 */
		if (int_cfg == 2) {
			data->handler[AIS2DW12_TEMP_TRIGGER] = handler;
			data->trigger[AIS2DW12_TEMP_TRIGGER] = trig;
		}
		break;
	case: SENSOR_TRIG_FIFO_WATERMARK
		data->handler[AIS2DW12_FIFO_THRES_TRIGGER] = handler;
		data->trigger[AIS2DW12_FIFO_THRES_TRIGGER] = trig;
		break;
	case: SENSOR_TRIG_FIFO_FULL
		data->handler[AIS2DW12_FIFO_FULL_TRIGGER] = handler;
		data->trigger[AIS2DW12_FIFO_FULL_TRIGGER] = trig;
		data->handler[AIS2DW12_FIFO_OVR_TRIGGER] = handler;
		data->trigger[AIS2DW12_FIFO_OVR_TRIGGER] = trig;
		break;
	default:
		LOG_ERR("Unsupported Tigger type");
		return -ENOTSUP;
	}

	rc = ais2dw12_init_interrupt(dev);
	if (rc < 0) {
		return rc;
	}

	return gpio_pin_interrupt_configure_dt(data->drdy_gpio,
					       GPIO_INT_EDGE_TO_ACTIVE);
}
