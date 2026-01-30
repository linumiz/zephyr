
/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_TC4XX_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_TC4XX_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <IfxPort_regdef.h>

enum gpio_tc4xx_input_mode {
	GPIO_TC4XX_INPUT_GPIO,
	GPIO_TC4XX_INPUT_PULL_DOWN,
	GPIO_TC4XX_INPUT_PULL_UP
};

enum gpio_tc4xx_irq_type {
	TC4XX_IRQ_TYPE_GTM = 1,
	TC4XX_IRQ_TYPE_ERU,
};

struct gpio_tc4xx_irq_source {
	uint32_t mux: 4;
	uint32_t ch: 4;
	uint32_t cls: 4;
	uint32_t type: 2;
	uint32_t pin: 4;
};

struct gpio_tc4xx_config {
	struct gpio_driver_config common;
	Ifx_P *base;
	uint8_t access_group;
	void (*config_func)(const struct device *dev);
	const struct gpio_tc4xx_irq_source *irq_sources;
    uint8_t irq_source_count;
	bool enable_gtm;
	bool enable_eru;
};

struct gpio_tc4xx_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_TC4XX_H_ */
