/*
 * Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 * Copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_cat1_pwm

#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>

#include <cy_tcpwm_pwm.h>
#include <cy_gpio.h>
#include <cy_sysclk.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm_ifx_cat1, CONFIG_PWM_LOG_LEVEL);


struct ifx_cat1_pwm_ch_config {
	const struct pinctrl_dev_config *pcfg;
	uint32_t divider_val;
	uint8_t channel_idx;
	uint8_t divider_type;
	uint8_t divider_sel;
};

struct ifx_cat1_pwm_config {
	uint32_t base;
	uint32_t grp_base;
	const struct ifx_cat1_pwm_ch_config *ch_cfg;
	uint8_t num_channels;
	bool resolution_32_bits;
#if (CY_IP_MXTCPWM_VERSION >= 2u)
	uint32_t clock_peri_group;
#endif
};

static int find_channel_idx(const struct ifx_cat1_pwm_config *config,
			    uint32_t channel)
{
	for (uint8_t i = 0; i < config->num_channels; i++) {
		if (config->ch_cfg[i].channel_idx == channel) {
			return (int)i;
		}
	}

	return -EINVAL;
}

static inline uint32_t get_ch_pwm_num(
	const struct ifx_cat1_pwm_config *config, uint8_t channel_idx)
{
#if (CY_IP_MXTCPWM_VERSION >= 2u)
	 TCPWM_Type *base = ( TCPWM_Type *)config->base;;
	if (config->grp_base >= (uint32_t)&base->GRP[2]) {
		return 512 + channel_idx;
	} else if (config->grp_base >= (uint32_t)&base->GRP[1]) {
		return 256 + channel_idx;
	}

	return channel_idx;
#else
	return ((config->grp_base - TCPWM0_BASE) +
		channel_idx * sizeof(TCPWM_GRP_CNT_Type)) /
	       sizeof(TCPWM_GRP_CNT_Type);
#endif
}

#if (CY_IP_MXTCPWM_VERSION >= 2u)
static inline uint32_t get_clk_connection( const struct ifx_cat1_pwm_config *config, uint8_t channel_idx)
{
        TCPWM_Type *base =( TCPWM_Type *) config->base;

        if (config->grp_base >= (uint32_t)&base->GRP[2]) {
#if (CY_IP_MXTCPWM_INSTANCES == 2u)
                if (config->base == TCPWM1_BASE) {
                        return PCLK_TCPWM1_CLOCKS512 + channel_idx;
                }
#endif
                return PCLK_TCPWM0_CLOCKS512 + channel_idx;
        } else if (config->grp_base >= (uint32_t)&base->GRP[1]) {
#if (CY_IP_MXTCPWM_INSTANCES == 2u)
                if (config->base == TCPWM1_BASE) {
                        return PCLK_TCPWM1_CLOCKS256 + channel_idx;
                }
#endif
                return PCLK_TCPWM0_CLOCKS256 + channel_idx;
        }

#if (CY_IP_MXTCPWM_INSTANCES == 2u)
        if (config->base == TCPWM1_BASE) {
                return PCLK_TCPWM1_CLOCKS0 + channel_idx;
        }
#endif
        return PCLK_TCPWM0_CLOCKS0 + channel_idx;
}
#endif

static int ifx_cat1_pwm_init(const struct device *dev)
{
	const struct ifx_cat1_pwm_config *config = dev->config;
	cy_en_tcpwm_status_t status;
	int ret;
	uint32_t clk_connection;
	const cy_stc_tcpwm_pwm_config_t pwm_config = {
		.pwmMode = CY_TCPWM_PWM_MODE_PWM,
		.clockPrescaler = CY_TCPWM_PWM_PRESCALER_DIVBY_1,
		.pwmAlignment = CY_TCPWM_PWM_LEFT_ALIGN,
		.runMode = CY_TCPWM_PWM_CONTINUOUS,
		.countInputMode = CY_TCPWM_INPUT_LEVEL,
		.countInput = CY_TCPWM_INPUT_1,
		.enableCompareSwap = true,
		.enablePeriodSwap = true,
	};

	for (uint8_t i = 0; i < config->num_channels; i++) {
		const struct ifx_cat1_pwm_ch_config *ch = &config->ch_cfg[i];
		uint32_t pwm_num = get_ch_pwm_num(config, ch->channel_idx);

#if (CY_IP_MXTCPWM_VERSION >= 2u)
		clk_connection = get_clk_connection(config, ch->channel_idx);
		/* Configure PWM clock */
		Cy_SysClk_PeriPclkDisableDivider(config->clock_peri_group,
						  ch->divider_type,
						  ch->divider_sel);

		Cy_SysClk_PeriPclkSetDivider(config->clock_peri_group,
					     ch->divider_type,
					     ch->divider_sel,
					     ch->divider_val);

		Cy_SysClk_PeriPclkEnableDivider(config->clock_peri_group,
						 ch->divider_type,
						 ch->divider_sel);

		Cy_SysClk_PeriPclkAssignDivider(clk_connection,
						 ch->divider_type,
						 ch->divider_sel);
#else
		/* Configure PWM clock */
		Cy_SysClk_PeriphDisableDivider(ch->divider_type,
					       ch->divider_sel);
		Cy_SysClk_PeriphSetDivider(ch->divider_type,
					   ch->divider_sel,
					   ch->divider_val);
		Cy_SysClk_PeriphEnableDivider(ch->divider_type,
					      ch->divider_sel);

		uint32_t addr_offset = (config->grp_base - TCPWM0_BASE) +
				       ch->channel_idx *
				       sizeof(TCPWM_GRP_CNT_Type);

		if (addr_offset < sizeof(TCPWM_GRP_Type)) {
			clk_connection = PCLK_TCPWM0_CLOCK_COUNTER_EN0 +
					 (addr_offset /
					  sizeof(TCPWM_GRP_CNT_Type));
		} else {
			addr_offset -= sizeof(TCPWM_GRP_Type);
			clk_connection = PCLK_TCPWM0_CLOCK_COUNTER_EN256 +
					 (addr_offset /
					  sizeof(TCPWM_GRP_CNT_Type));
		}

		Cy_SysClk_PeriphAssignDivider(clk_connection,
					      ch->divider_type,
					      ch->divider_sel);
#endif

		ret = pinctrl_apply_state(ch->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}

		status = Cy_TCPWM_PWM_Init((TCPWM_Type *)config->base, pwm_num,
					   &pwm_config);
		if (status != CY_TCPWM_SUCCESS) {
			return -ENOTSUP;
		}
	}

	return 0;
}

static int ifx_cat1_pwm_set_cycles(const struct device *dev, uint32_t channel,
				   uint32_t period_cycles, uint32_t pulse_cycles,
				   pwm_flags_t flags)
{
	const struct ifx_cat1_pwm_config *config = dev->config;
	int idx;

	idx = find_channel_idx(config, channel);
	if (idx < 0) {
		return -EINVAL;
	}

	const struct ifx_cat1_pwm_ch_config *ch = &config->ch_cfg[idx];
	uint32_t pwm_num = get_ch_pwm_num(config, ch->channel_idx);

	if (!config->resolution_32_bits &&
	    ((period_cycles > UINT16_MAX) || (pulse_cycles > UINT16_MAX))) {
		/* 16-bit resolution */
		if (period_cycles > UINT16_MAX) {
			LOG_ERR("Period cycles more than 16-bits (%u)",
				period_cycles);
		}
		if (pulse_cycles > UINT16_MAX) {
			LOG_ERR("Pulse cycles more than 16-bits (%u)",
				pulse_cycles);
		}
		return -EINVAL;
	}

	if ((period_cycles == 0) || (pulse_cycles == 0)) {
		Cy_TCPWM_PWM_Disable((TCPWM_Type *)config->base, pwm_num);
	} else {
		Cy_TCPWM_PWM_SetPeriod1((TCPWM_Type *)config->base, pwm_num,
					period_cycles);
		Cy_TCPWM_PWM_SetCompare0BufVal((TCPWM_Type *)config->base, pwm_num,
						pulse_cycles);

		Cy_TCPWM_TriggerCaptureOrSwap_Single((TCPWM_Type *)config->base, pwm_num);

		if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_INVERTED) {
			TCPWM_GRP_CNT_Type *reg = (TCPWM_GRP_CNT_Type *)(config->grp_base + ch->channel_idx * sizeof(TCPWM_GRP_CNT_Type));

			reg->CTRL &=
				~TCPWM_GRP_CNT_V2_CTRL_QUAD_ENCODING_MODE_Msk;
			reg->CTRL |=
				_VAL2FLD(TCPWM_GRP_CNT_V2_CTRL_QUAD_ENCODING_MODE,
					 CY_TCPWM_PWM_INVERT_ENABLE);
		}

		Cy_TCPWM_PWM_Enable((TCPWM_Type *)config->base, pwm_num);
		Cy_TCPWM_TriggerStart_Single((TCPWM_Type *)config->base, pwm_num);
	}

	return 0;
}

static int ifx_cat1_pwm_get_cycles_per_sec(const struct device *dev,
					   uint32_t channel, uint64_t *cycles)
{
	const struct ifx_cat1_pwm_config *config = dev->config;
	int idx;

	idx = find_channel_idx(config, channel);
	if (idx < 0) {
		return -EINVAL;
	}

	const struct ifx_cat1_pwm_ch_config *ch = &config->ch_cfg[idx];

#if (CY_IP_MXTCPWM_VERSION >= 2u)
	*cycles = Cy_SysClk_PeriPclkGetFrequency(config->clock_peri_group,
						  ch->divider_type,
						  ch->divider_sel);
#else
	*cycles = Cy_SysClk_PeriphGetFrequency(ch->divider_type,
						ch->divider_sel);
#endif
	return 0;
}

static DEVICE_API(pwm, ifx_cat1_pwm_api) = {
	.set_cycles = ifx_cat1_pwm_set_cycles,
	.get_cycles_per_sec = ifx_cat1_pwm_get_cycles_per_sec,
};

#define PWM_CH_PINCTRL_DEFINE(child) PINCTRL_DT_DEFINE(child);

#define PWM_CH_CONFIG_ENTRY(child)                                                                 \
	{                                                                                          \
		.channel_idx = DT_REG_ADDR(child),                                                 \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(child),                                          \
		.divider_type = DT_PROP(child, divider_type),                                      \
		.divider_sel = DT_PROP(child, divider_sel),                                        \
		.divider_val = DT_PROP(child, divider_val),                                        \
	},

#define PWM_COUNT_CH(child) + 1
#define PWM_NUM_CH(n) (0 DT_INST_FOREACH_CHILD_STATUS_OKAY(n, PWM_COUNT_CH))

#if (CY_IP_MXTCPWM_VERSION >= 2u)
#define INFINEON_CAT1_PWM_INIT(n)                                                                  \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(n, PWM_CH_PINCTRL_DEFINE)                                \
                                                                                                   \
	static const struct ifx_cat1_pwm_ch_config                                                 \
		pwm_cat1_ch_cfg_##n[] = {                                                          \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(n, PWM_CH_CONFIG_ENTRY)                          \
	};                                                                                         \
                                                                                                   \
	static const struct ifx_cat1_pwm_config pwm_cat1_config_##n = {                            \
		.base = DT_REG_ADDR(DT_PARENT(DT_INST_PARENT(n))),                                                   \
		.grp_base = DT_INST_REG_ADDR(n),                                                   \
		.num_channels = PWM_NUM_CH(n),                                                     \
		.resolution_32_bits =                                                              \
			(DT_INST_PROP(n, resolution) == 32) ? true : false,                        \
		.ch_cfg = pwm_cat1_ch_cfg_##n,                                                     \
		.clock_peri_group = DT_INST_PROP(n, ifx_peri_group),                               \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ifx_cat1_pwm_init, NULL, NULL,                                    \
			      &pwm_cat1_config_##n, POST_KERNEL,                                   \
			      CONFIG_PWM_INIT_PRIORITY, &ifx_cat1_pwm_api);
#else
#define INFINEON_CAT1_PWM_INIT(n)                                                                  \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(n, PWM_CH_PINCTRL_DEFINE)                                \
                                                                                                   \
	static const struct ifx_cat1_pwm_ch_config                                                 \
		pwm_cat1_ch_cfg_##n[] = {                                                          \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(n, PWM_CH_CONFIG_ENTRY)                          \
	};                                                                                         \
                                                                                                   \
	static const struct ifx_cat1_pwm_config pwm_cat1_config_##n = {                            \
		.grp_base = DT_INST_REG_ADDR(n),                                                   \
		.num_channels = PWM_NUM_CH(n),                                                     \
		.resolution_32_bits =                                                              \
			(DT_INST_PROP(n, resolution) == 32) ? true : false,                        \
		.ch_cfg = pwm_cat1_ch_cfg_##n,                                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ifx_cat1_pwm_init, NULL, NULL,                                   \
			      &pwm_cat1_config_##n, POST_KERNEL,                                   \
			      CONFIG_PWM_INIT_PRIORITY, &ifx_cat1_pwm_api);
#endif
DT_INST_FOREACH_STATUS_OKAY(INFINEON_CAT1_PWM_INIT)
