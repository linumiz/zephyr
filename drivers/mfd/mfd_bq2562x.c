/*
 * Copyright 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_bq2562x

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mfd/mfd_bq2562x.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mfd_bq2562x, CONFIG_MFD_LOG_LEVEL);

#define BQ2562X_PART_INFO        0x38

/* REG0x38_Part_Information */
#define BQ2562X_PART_NO_MASK GENMASK(5, 3)

struct mfd_bq2562x_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec int_gpio;
};

struct mfd_bq2562x_data {
	const struct device *dev;
	struct gpio_callback gpio_cb;
	struct k_work int_routine_work;
	sys_slist_t callback_list;
};

enum mfd_bq2562x_id {
	BQ25620,
	BQ25622,
};

int mfd_bq2562x_reg_update_byte_dt(const struct device *dev, uint8_t reg_addr, uint8_t mask,
				   uint8_t value)
{
	const struct mfd_bq2562x_config *config = dev->config;

	return i2c_reg_update_byte_dt(&config->i2c, reg_addr, mask, value);

}

int mfd_bq2562x_reg_read_byte_dt(const struct device *dev, uint8_t reg_addr, uint8_t *value)
{
	const struct mfd_bq2562x_config *config = dev->config;

	return i2c_reg_read_byte_dt(&config->i2c, reg_addr, value);
}

int mfd_bq2562x_burst_read_dt(const struct device *dev, uint8_t start_addr,
			      uint8_t *buf, uint32_t num_bytes)
{
	const struct mfd_bq2562x_config *config = dev->config;

	return i2c_burst_read_dt(&config->i2c, start_addr, buf, num_bytes);

}

int mfd_bq2562x_burst_write_dt(const struct device *dev, uint8_t start_addr,
			       uint8_t *buf, uint32_t num_bytes)
{
	const struct mfd_bq2562x_config *config = dev->config;

	return i2c_burst_write_dt(&config->i2c, start_addr, buf, num_bytes);
}

int mfd_bq2562x_enable_interrupt_pin(const struct device *dev, bool enabled)
{
	const struct mfd_bq2562x_config *const config = dev->config;
	gpio_flags_t flags;
	int ret;

	flags = enabled ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;

	ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, flags);
	if (ret < 0) {
		LOG_ERR("Could not %s interrupt GPIO callback: %d", enabled ? "enable" : "disable",
			ret);
	}

	return ret;
}

void mfd_bq2562x_register_interrupt_callback(const struct device *mfd,
					     struct bq2562x_mfd_callback *callback)
{
	struct mfd_bq2562x_data *data = mfd->data;

	sys_slist_append(&data->callback_list, &callback->node);
}

static void mfd_bq2562x_int_work_handler(struct k_work *work)
{
	struct mfd_bq2562x_data *data = CONTAINER_OF(work, struct mfd_bq2562x_data,
						     int_routine_work);
	struct bq2562x_mfd_callback *cb_entry;

	SYS_SLIST_FOR_EACH_CONTAINER(&data->callback_list, cb_entry, node) {
		cb_entry->cb(cb_entry->dev);
	}

	(void)mfd_bq2562x_enable_interrupt_pin(data->dev, true);
}

static void mfd_bq2562x_gpio_callback(const struct device *dev, struct gpio_callback *cb,
				      uint32_t pins)
{
	struct mfd_bq2562x_data *data = CONTAINER_OF(cb, struct mfd_bq2562x_data, gpio_cb);
	int ret;

	(void)mfd_bq2562x_enable_interrupt_pin(data->dev, false);

	ret = k_work_submit(&data->int_routine_work);
	if (ret < 0) {
		LOG_ERR("Could not submit int work: %d", ret);
	}
}

static int mfd_bq2562x_configure_interrupt(const struct device *dev)
{
	const struct mfd_bq2562x_config *const config = dev->config;
	struct mfd_bq2562x_data *data = dev->data;
	int ret;

	k_work_init(&data->int_routine_work, mfd_bq2562x_int_work_handler);
	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("Interrupt GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure interrupt GPIO");
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, mfd_bq2562x_gpio_callback, BIT(config->int_gpio.pin));
	ret = gpio_add_callback_dt(&config->int_gpio, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Could not add interrupt GPIO callback");
		return ret;
	}

	(void)mfd_bq2562x_enable_interrupt_pin(dev, true);

	return 0;
}

static int mfd_bq2562x_init(const struct device *dev)
{
	const struct mfd_bq2562x_config *const config = dev->config;
	struct mfd_bq2562x_data *data = dev->data;
	uint8_t val;
	int ret;

	data->dev = dev;

	if (!i2c_is_ready_dt(&config->i2c)) {
		return -ENODEV;
	}

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_PART_INFO, &val);
	if (ret) {
		return ret;
	}

	val = FIELD_GET(BQ2562X_PART_NO_MASK, val);
	if (val == BQ25622) {
		return -ENOTSUP;
	}

	if (config->int_gpio.port != NULL) {
		ret = mfd_bq2562x_configure_interrupt(dev);
		if (ret) {
			return ret;
		}
	}

	return ret;
}

#define BQ2562X_INIT(n)										\
												\
	static const struct mfd_bq2562x_config mfd_bq2562x_config_##n = {			\
		.i2c = I2C_DT_SPEC_INST_GET(n),							\
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {}),				\
	};											\
												\
	static struct mfd_bq2562x_data mfd_bq2562x_data_##n;					\
												\
	DEVICE_DT_INST_DEFINE(n, mfd_bq2562x_init, NULL, &mfd_bq2562x_data_##n,			\
			      &mfd_bq2562x_config_##n, POST_KERNEL,				\
			      CONFIG_MFD_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(BQ2562X_INIT)

