/*
 * Copyright (c) 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_ifx_cat1.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <cy_sysclk.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ifx_clock_control, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#define DT_DRV_COMPAT	infineon_cat1_clock

#define IFX_PLL_ENABLETIMEOUT	10000 /* 10 ms */

struct ifx_clock_config {
	uint32_t clock_frequency;
	uint8_t predivider;
	uint8_t clock_instance;
	uint8_t p_div;
	uint8_t q_div;
	uint8_t output_div;
	uint8_t pll_instance;
	uint8_t pll_source;
};

struct ifx_clock_data {
	struct k_mutex lock;
};

static int ifx_clock_get_rate(const struct device *dev,
			      clock_control_subsys_t sys, uint32_t *rate)
{
	const struct ifx_clock_config *cfg = dev->config;
	struct ifx_clock_data *data = dev->data;

	if (rate == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	*rate = cfg->clock_frequency;
	k_mutex_unlock(&data->lock);

	return 0;
}

static int ifx_setup_pll(const struct device *dev)
{
	int ret;
	const struct ifx_clock_config *cfg = dev->config;
	cy_stc_pll_manual_config_t pll_config = {0};

	if (cfg->pll_source == -1) {
		return -EINVAL;
	}
	ret = Cy_SysClk_ClkPathSetSource(cfg->pll_instance, cfg->pll_source);
	if (ret != 0) {
		return ret;
	}

	pll_config.feedbackDiv = cfg->p_div;
	pll_config.referenceDiv = cfg->q_div;
	pll_config.outputDiv = cfg->output_div;
	pll_config.lfMode = false;
	pll_config.outputMode = CY_SYSCLK_FLLPLL_OUTPUT_AUTO;

	ret = Cy_SysClk_PllDisable(cfg->pll_instance);
	if (ret != 0) {
		return ret;
	}

	ret = Cy_SysClk_PllManualConfigure(cfg->pll_instance,
					   &pll_config);
	if (ret != 0) {
		return ret;
	}

	ret = Cy_SysClk_PllEnable(cfg->pll_instance,
				  IFX_PLL_ENABLETIMEOUT);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

static int ifx_config_hfclk(const struct device *dev)
{
	int ret;
	const struct ifx_clock_config *cfg = dev->config;

	ret = Cy_SysClk_ClkHfSetSource(cfg->clock_instance, cfg->pll_instance);
	if (ret != 0) {
		return ret;
	}

	ret = Cy_SysClk_ClkHfSetDivider(cfg->clock_instance, cfg->predivider);
	if (ret != 0) {
		return ret;
	}

	ret = Cy_SysClk_ClkHfDirectSel(cfg->clock_instance, false);
	if (ret != 0){
		return ret;
	}

	return Cy_SysClk_ClkHfEnable(cfg->clock_instance);
}

static int ifx_clock_init(const struct device *dev)
{
	int ret;
	struct ifx_clock_data *data = dev->data;

	k_mutex_init(&data->lock);
	ret = ifx_setup_pll(dev);
	if (ret != 0) {
		LOG_ERR("PLL config failed %d\n", ret);
		return ret;
	}

	ret = ifx_config_hfclk(dev);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static DEVICE_API(clock_control, clock_control_ifx_cat1_api) = {
	.get_rate = ifx_clock_get_rate,
};

#define IFX_CLK_INIT(idx)							\
	static const struct ifx_clock_config ifx_clock_config_##idx = {		\
		.clock_frequency = DT_INST_PROP(idx, clock_frequency),		\
		.clock_instance = DT_INST_PROP(idx, ifx_hf_instance),		\
		.pll_instance = DT_INST_PROP_BY_PHANDLE(idx, clocks, ifx_pll_instance),	\
		.pll_source = DT_PROP_OR(DT_CLOCKS_CTLR(DT_INST_CLOCKS_CTLR(idx)), clock_block, -1), \
		.p_div = DT_INST_PROP_BY_PHANDLE(idx, clocks, p_div),		\
		.q_div = DT_INST_PROP_BY_PHANDLE(idx, clocks, q_div),		\
		.output_div = DT_INST_PROP_BY_PHANDLE(idx, clocks, output_div),	\
		.predivider = DT_INST_PROP(idx, ifx_hf_predivider),		\
	};                                                                      \
										\
	struct ifx_clock_data data_##idx;					\
										\
	DEVICE_DT_INST_DEFINE(idx, &ifx_clock_init, NULL, &data_##idx,		\
			      &ifx_clock_config_##idx, PRE_KERNEL_1,		\
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,		\
			      &clock_control_ifx_cat1_api);

DT_INST_FOREACH_STATUS_OKAY(IFX_CLK_INIT)
