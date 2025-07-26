/*
 * Copyright (c) 2025 Texas Instruments
 * Copyright (c) 2025 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DAC DT_NODELABEL(dac)

#include <zephyr/kernel.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dac_mspm0_wrapper);

/**
 * @brief Get the DAC device instance
 */
static const struct device *const dac_dev = DEVICE_DT_GET(DAC);

int dac_mspm0_channel_setup(const struct dac_channel_cfg *cfg)
{
	int ret;

	if (!device_is_ready(dac_dev)) {
		LOG_ERR("DAC device not ready");
		return -ENODEV;
	}

	ret = dac_channel_setup(dac_dev, cfg);
	if (ret) {
		LOG_ERR("Failed to setup DAC channel: %d", ret);
		return ret;
	}

	LOG_DBG("DAC channel setup successful");

	return 0;
}

int dac_mspm0_write_value(uint8_t channel, uint32_t value)
{
	int ret;

	if (!device_is_ready(dac_dev)) {
		LOG_ERR("DAC device not ready");
		return -ENODEV;
	}

	ret = dac_write_value(dac_dev, channel, value);
	if (ret) {
		LOG_ERR("Failed to write DAC value %d on channel %d: %d", value, channel, ret);
		return ret;
	}

	LOG_DBG("DAC value %d written to channel %d", value, channel);

	return 0;
}
