/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/infineon_peri_clock.h>
#include <zephyr/dt-bindings/clock/infineon_clock.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <cy_sysclk.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ifx_peri_clock_control, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#define DT_DRV_COMPAT			infineon_peri_clock

#define IFX_FRACTION_DIVIDER		32
#define IFX_PERI_CLK_MAX_DIVIDER	(IFX_PERI_DIV_24_5 + 1)

struct ifx_peri_clock_config {
	uint32_t src_clk_freq;
	uint32_t peri_clk_inst;
	uint8_t max_dividers[IFX_PERI_CLK_MAX_DIVIDER];
};

struct ifx_peri_clock_data {
	struct k_mutex lock;
};

static int clock_control_ifx_peri_clock_get_rate(const struct device *dev,
						 clock_control_subsys_t sys,
						 uint32_t *rate)
{
	struct infineon_sys_clock *clk = (struct infineon_sys_clock *)sys;
	const struct ifx_peri_clock_config *cfg = dev->config;
	struct ifx_peri_clock_data *data = dev->data;
	uint32_t frac_div = 0;
	uint32_t int_div;

	if (rate == NULL) {
		return -EINVAL;
	}

	if (clk->divider_inst > cfg->max_dividers[clk->divider_type]) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	switch (clk->divider_type) {
	case IFX_PERI_DIV_8:
	case IFX_PERI_DIV_16:
		int_div = Cy_SysClk_PeriPclkGetDivider(cfg->peri_clk_inst,
						       clk->divider_type,
						       clk->divider_inst);
		break;
	case IFX_PERI_DIV_24_5:
	case IFX_PERI_DIV_16_5:
		Cy_SysClk_PeriPclkGetFracDivider(cfg->peri_clk_inst,
						 clk->divider_type,
						 clk->divider_inst,
						 &int_div, &frac_div);
		break;
	default:
		goto out;
	}

	if (frac_div) {
		frac_div = frac_div / 100;
	}

	*rate = cfg->src_clk_freq / (int_div + frac_div);

out:
	k_mutex_unlock(&data->lock);

	return 0;
}

static int clock_control_ifx_peri_clock_set_rate(const struct device *dev,
						 clock_control_subsys_t sys,
						 clock_control_subsys_rate_t rate)
{
	struct infineon_sys_clock *clk = (struct infineon_sys_clock *)sys;
	const struct ifx_peri_clock_config *cfg = dev->config;
	struct ifx_peri_clock_data *data = dev->data;
	int *clk_rate = (int32_t *)rate;
	uint32_t clock_id;
	uint32_t int_div;
	int ret;

	if ((rate == NULL) || (sys == NULL)) {
		return -EINVAL;
	}

	if (clk->divider_inst > cfg->max_dividers[clk->divider_type]) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	int_div = (cfg->src_clk_freq / *clk_rate);
	ret = Cy_SysClk_PeriPclkDisableDivider(cfg->peri_clk_inst,
					       clk->divider_type,
					       clk->divider_inst);
	if (ret != 0) {
		goto out;
	}

	switch (clk->divider_type) {
	case IFX_PERI_DIV_8:
	case IFX_PERI_DIV_16:
		ret = Cy_SysClk_PeriPclkSetDivider(cfg->peri_clk_inst,
						   clk->divider_type,
						   clk->divider_inst,
						   int_div - 1);
		break;
	case IFX_PERI_DIV_16_5:
	case IFX_PERI_DIV_24_5:
		uint32_t frac_div = (((float)cfg->src_clk_freq / (float) *clk_rate) \
					- int_div) * IFX_FRACTION_DIVIDER;

		ret = Cy_SysClk_PeriPclkSetFracDivider(cfg->peri_clk_inst,
						       clk->divider_type,
						       clk->divider_inst,
						       int_div - 1, frac_div);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	if (ret != 0) {
		goto out;
	}

        ret = Cy_SysClk_PeriPclkEnableDivider(cfg->peri_clk_inst,
					      clk->divider_type,
					      clk->divider_inst);
        if (ret != 0) {
		goto out;
        }

	clock_id = cfg->peri_clk_inst | clk->root_clk_id;
        ret = Cy_SysClk_PeriPclkAssignDivider(clock_id, clk->divider_type,
                                              clk->divider_inst);

out:
	k_mutex_unlock(&data->lock);

	return ret;
}

static int clock_control_ifx_peri_clock_init(const struct device *dev)
{
	struct ifx_peri_clock_data *data = dev->data;

	k_mutex_init(&data->lock);

	return 0;
}

static DEVICE_API(clock_control, ifx_peri_clock_control_api) = {
	.get_rate = clock_control_ifx_peri_clock_get_rate,
	.set_rate = clock_control_ifx_peri_clock_set_rate,
};

#define FILL_DIVIDER_INST(inst) 								\
	.max_dividers[IFX_PERI_DIV_8] = DT_INST_PROP_OR(inst, ifx_8_bit_max_divider, 0),	\
	.max_dividers[IFX_PERI_DIV_16] = DT_INST_PROP_OR(inst, ifx_16_bit_max_divider, 0),	\
	.max_dividers[IFX_PERI_DIV_16_5] = DT_INST_PROP_OR(inst, ifx_16_5_bit_max_divider, 0),	\
	.max_dividers[IFX_PERI_DIV_24_5] = DT_INST_PROP_OR(inst, ifx_8_bit_max_divider, 0),

#define IFX_PERI_CLOCK_INIT(inst)								\
	static const struct ifx_peri_clock_config clk_cfg_##inst = {				\
		.src_clk_freq = DT_INST_PROP_BY_PHANDLE(inst, clocks, clock_frequency),		\
		.peri_clk_inst = DT_INST_PROP_OR(inst, ifx_peri_clk_inst, 0),			\
		FILL_DIVIDER_INST(inst)								\
	};											\
												\
	struct ifx_peri_clock_data clk_data_##inst;						\
												\
	DEVICE_DT_INST_DEFINE(inst, clock_control_ifx_peri_clock_init,				\
			      NULL, &clk_data_##inst, &clk_cfg_##inst,				\
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,			\
			      &ifx_peri_clock_control_api);

DT_INST_FOREACH_STATUS_OKAY(IFX_PERI_CLOCK_INIT)
