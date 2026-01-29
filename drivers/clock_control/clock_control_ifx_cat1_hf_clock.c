/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/toolchain.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <cy_sysclk.h>


#define DT_DRV_COMPAT infineon_cat1_hf_clock
LOG_MODULE_REGISTER(ifx_cat1_hf_clock, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct ifx_cat1_hf_clock_config {
	uint32_t id;
};

struct ifx_cat1_hf_clock_data {
};

static int ifx_cat1_hf_clock_get_rate(const struct device *dev, clock_control_subsys_t sub_system,
				      uint32_t *rate)
{
	;

	const struct ifx_cat1_hf_clock_config *config = dev->config;

	if (rate == NULL) {
		return -EINVAL;
	}

	*rate = Cy_SysClk_ClkHfGetFrequency(config->id);

	return 0;
}

static int ifx_cat1_hf_clock_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static const struct clock_control_driver_api ifx_cat1_hf_clock_api = {
	.get_rate = ifx_cat1_hf_clock_get_rate,
};

#define IFX_CAT1_HF_CLOCK_INIT(inst)                                                               \
                                                                                                   \
	static const struct ifx_cat1_hf_clock_config ifx_cat1_hf_clock_config_##inst = {           \
		.id = DT_INST_REG_ADDR(inst)};                                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, ifx_cat1_hf_clock_init, NULL, NULL,                            \
			      &ifx_cat1_hf_clock_config_##inst, PRE_KERNEL_1,                      \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &ifx_cat1_hf_clock_api);

DT_INST_FOREACH_STATUS_OKAY(IFX_CAT1_HF_CLOCK_INIT)
