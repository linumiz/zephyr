/*
 * copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_tcpwm_tgpio

#include <zephyr/device.h>
#include <zephyr/drivers/misc/timeaware_gpio/timeaware_gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tgpio_ifx_tcpwm, CONFIG_TGPIO_LOG_LEVEL);

#include <cy_tcpwm_counter.h>
#include <cy_tcpwm_pwm.h>
#include <cy_sysclk.h>

struct tgpio_ifx_ch_config {
	const struct pinctrl_dev_config *pcfg;
	uint16_t irq_num;
	uint8_t channel_idx;
	uint8_t irq_priority;
};

struct tgpio_ifx_ch_data {
	TCPWM_Type *base;
	uint32_t last_timestamp;
	uint16_t event_count;
	uint16_t cnt_idx;
};

struct tgpio_ifx_config {
	uint32_t base;
	uint32_t grp_base;
	const struct tgpio_ifx_ch_config *ch_cfg;
	uint32_t clock_peri_group;
	uint32_t divider_val;
	uint8_t num_channels;
	uint8_t divider_type;
	uint8_t divider_sel;
};

static int find_ch(const struct tgpio_ifx_config *cfg, uint32_t pin)
{
	for (uint8_t i = 0; i < cfg->num_channels; i++) {
		if (cfg->ch_cfg[i].channel_idx == pin) {
			return (int)i;
		}
	}

	return -EINVAL;
}

static inline uint32_t get_cnt_idx(const struct tgpio_ifx_config *cfg,
				   uint8_t channel_idx)
{
	TCPWM_Type *base = ( TCPWM_Type *)cfg->base;

	if (cfg->grp_base >= (uint32_t)&base->GRP[2]) {
		return 512 + channel_idx;
	} else if (cfg->grp_base >= (uint32_t)&base->GRP[1]) {
		return 256 + channel_idx;
	}

	return channel_idx;
}

#if (CY_IP_MXTCPWM_VERSION >= 2u)
static inline uint32_t get_clk_connection(const struct tgpio_ifx_config *cfg,
					  uint8_t channel_idx)
{
	TCPWM_Type *base = ( TCPWM_Type *)cfg->base;

	if (cfg->grp_base >= (uint32_t)&base->GRP[2]) {
#if (CY_IP_MXTCPWM_INSTANCES == 2u)
		if (cfg->base == TCPWM1_BASE) {
			return PCLK_TCPWM1_CLOCKS512 + channel_idx;
		}
#endif
		return PCLK_TCPWM0_CLOCKS512 + channel_idx;
	} else if (cfg->grp_base >= (uint32_t)&base->GRP[1]) {
#if (CY_IP_MXTCPWM_INSTANCES == 2u)
		if (cfg->base == TCPWM1_BASE) {
			return PCLK_TCPWM1_CLOCKS256 + channel_idx;
		}
#endif
		return PCLK_TCPWM0_CLOCKS256 + channel_idx;
	}

#if (CY_IP_MXTCPWM_INSTANCES == 2u)
	if (cfg->base == TCPWM1_BASE) {
		return PCLK_TCPWM1_CLOCKS0 + channel_idx;
	}
#endif
	return PCLK_TCPWM0_CLOCKS0 + channel_idx;
}
#endif

static void tgpio_ifx_ch_isr(const void *arg)
{
	struct tgpio_ifx_ch_data *chd = (struct tgpio_ifx_ch_data *)arg;

	uint32_t intr = Cy_TCPWM_GetInterruptStatusMasked(chd->base,
							   chd->cnt_idx);

	if (intr & CY_TCPWM_INT_ON_CC0) {
		chd->last_timestamp = Cy_TCPWM_Counter_GetCapture(
						chd->base, chd->cnt_idx);
		chd->event_count++;
	}

	Cy_TCPWM_ClearInterrupt(chd->base, chd->cnt_idx, CY_TCPWM_INT_ON_CC0);
}

static int tgpio_ifx_get_time(const struct device *dev, uint64_t *t)
{
	const struct tgpio_ifx_config *cfg = dev->config;
	struct tgpio_ifx_ch_data *ch = dev->data;

	*t = (uint64_t)Cy_TCPWM_Counter_GetCounter( (TCPWM_Type *)cfg->base,
						    ch[0].cnt_idx);
	return 0;
}

static int tgpio_ifx_cyc_per_sec(const struct device *dev, uint32_t *cyc)
{
	const struct tgpio_ifx_config *cfg = dev->config;

	*cyc = Cy_SysClk_PeriPclkGetFrequency(cfg->clock_peri_group,
					       cfg->divider_type,
					       cfg->divider_sel);
	return 0;
}

static int tgpio_ifx_pin_disable(const struct device *dev, uint32_t pin)
{
	const struct tgpio_ifx_config *cfg = dev->config;
	struct tgpio_ifx_ch_data *ch = dev->data;
	TCPWM_Type *base = ( TCPWM_Type *)cfg->base;
	int idx;

	idx = find_ch(cfg, pin);
	if (idx < 0) {
		return -EINVAL;
	}

	Cy_TCPWM_SetInterruptMask(base, ch[idx].cnt_idx, 0);
	Cy_TCPWM_TriggerStopOrKill_Single(base, ch[idx].cnt_idx);
	Cy_TCPWM_Disable_Single(base, ch[idx].cnt_idx);
	return 0;
}

static int tgpio_ifx_set_per_out(const struct device *dev, uint32_t pin,
				 uint64_t start_time, uint64_t period,
				 bool periodic)
{
	const struct tgpio_ifx_config *cfg = dev->config;
	struct tgpio_ifx_ch_data *ch = dev->data;
	TCPWM_Type *base =  (TCPWM_Type *)cfg->base;
	int idx;
	int ret;
	uint16_t cnt;

	ARG_UNUSED(start_time);

	idx = find_ch(cfg, pin);
	if (idx < 0) {
		return -EINVAL;
	}

	if (period > UINT32_MAX) {
		return -EINVAL;
	}

	cnt = ch[idx].cnt_idx;

	ret = pinctrl_apply_state(cfg->ch_cfg[idx].pcfg,
				  PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	Cy_TCPWM_TriggerStopOrKill_Single(base, cnt);
	Cy_TCPWM_Disable_Single(base, cnt);

	cy_stc_tcpwm_pwm_config_t pwm_cfg = {
		.pwmMode = CY_TCPWM_PWM_MODE_PWM,
		.clockPrescaler = CY_TCPWM_PWM_PRESCALER_DIVBY_1,
		.pwmAlignment = CY_TCPWM_PWM_LEFT_ALIGN,
		.runMode = periodic ? CY_TCPWM_PWM_CONTINUOUS :
				      CY_TCPWM_PWM_ONESHOT,
		.countInputMode = CY_TCPWM_INPUT_LEVEL,
		.countInput = CY_TCPWM_INPUT_1,
		.period0 = (uint32_t)period - 1,
		.compare0 = (uint32_t)(period / 2),
		.enablePeriodSwap = false,
		.enableCompareSwap = false,
		.interruptSources = 0,
	};

	if (Cy_TCPWM_PWM_Init(base, cnt, &pwm_cfg) != CY_TCPWM_SUCCESS) {
		return -EIO;
	}

	Cy_TCPWM_PWM_Enable(base, cnt);
	Cy_TCPWM_TriggerStart_Single(base, cnt);

	ch[idx].event_count = 0;
	return 0;
}

static int tgpio_ifx_config_ext_ts(const struct device *dev, uint32_t pin,
				   uint32_t edge)
{
	const struct tgpio_ifx_config *cfg = dev->config;
	struct tgpio_ifx_ch_data *ch = dev->data;
	TCPWM_Type *base =  (TCPWM_Type *)cfg->base;
	int idx;
	int ret;
	uint32_t edge_sel;

	idx = find_ch(cfg, pin);
	if (idx < 0) {
		return -EINVAL;
	}

	ret = pinctrl_apply_state(cfg->ch_cfg[idx].pcfg,
				  PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ch[idx].last_timestamp = 0;
	ch[idx].event_count = 0;

	if (edge == TGPIO_FALLING_EDGE) {
		edge_sel = CY_TCPWM_INPUT_FALLINGEDGE;
	} else if (edge == TGPIO_TOGGLE_EDGE) {
		edge_sel = CY_TCPWM_INPUT_EITHEREDGE;
	} else {
		edge_sel = CY_TCPWM_INPUT_RISINGEDGE;
	}

	Cy_TCPWM_InputTriggerSetup(base, ch[idx].cnt_idx,
				   CY_TCPWM_INPUT_TR_CAPTURE0,
				   edge_sel, CY_TCPWM_INPUT_TRIG_0);

	Cy_TCPWM_ClearInterrupt(base, ch[idx].cnt_idx,
				 CY_TCPWM_INT_ON_CC0);
	Cy_TCPWM_SetInterruptMask(base, ch[idx].cnt_idx,
				   CY_TCPWM_INT_ON_CC0);

	return 0;
}

static int tgpio_ifx_read_ts_ec(const struct device *dev, uint32_t pin,
				uint64_t *ts, uint64_t *ec)
{
	const struct tgpio_ifx_config *cfg = dev->config;
	struct tgpio_ifx_ch_data *ch = dev->data;
	int idx;

	idx = find_ch(cfg, pin);
	if (idx < 0) {
		return -EINVAL;
	}

	*ts = (uint64_t)ch[idx].last_timestamp;
	*ec = (uint64_t)ch[idx].event_count;
	return 0;
}

static const struct tgpio_driver_api tgpio_ifx_api = {
	.pin_disable = tgpio_ifx_pin_disable,
	.get_time = tgpio_ifx_get_time,
	.cyc_per_sec = tgpio_ifx_cyc_per_sec,
	.set_perout = tgpio_ifx_set_per_out,
	.config_ext_ts = tgpio_ifx_config_ext_ts,
	.read_ts_ec = tgpio_ifx_read_ts_ec,
};

static int tgpio_ifx_init(const struct device *dev)
{
	const struct tgpio_ifx_config *cfg = dev->config;
	const struct tgpio_ifx_ch_config *cc;
	struct tgpio_ifx_ch_data *chd; 
	struct tgpio_ifx_ch_data *ch = dev->data;
	TCPWM_Type *base = ( TCPWM_Type *)cfg->base;
	uint32_t period;
	uint32_t clk;
	int ret;

	period = (cfg->grp_base >= (uint32_t)&base->GRP[2]) ?
		 UINT32_MAX : UINT16_MAX;

	Cy_SysClk_PeriPclkDisableDivider(cfg->clock_peri_group,
					  cfg->divider_type,
					  cfg->divider_sel);
	Cy_SysClk_PeriPclkSetDivider(cfg->clock_peri_group,
				     cfg->divider_type,
				     cfg->divider_sel,
				     cfg->divider_val);

	for (uint8_t i = 0; i < cfg->num_channels; i++) {
		cc = &cfg->ch_cfg[i];
		chd = &ch[i];

		chd->cnt_idx = get_cnt_idx(cfg, cc->channel_idx);
		chd->last_timestamp = 0;
		chd->event_count = 0;
		chd->base = base;

		clk = get_clk_connection(cfg, cc->channel_idx);

		Cy_SysClk_PeriPclkAssignDivider(clk, cfg->divider_type,
						 cfg->divider_sel);

		ret = pinctrl_apply_state(cc->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}

		cy_stc_tcpwm_counter_config_t cnt_cfg = {
			.period = period,
			.clockPrescaler = CY_TCPWM_COUNTER_PRESCALER_DIVBY_1,
			.runMode = CY_TCPWM_COUNTER_CONTINUOUS,
			.countDirection = CY_TCPWM_COUNTER_COUNT_UP,
			.compareOrCapture = CY_TCPWM_COUNTER_MODE_CAPTURE,
			.countInputMode = CY_TCPWM_INPUT_LEVEL,
			.countInput = CY_TCPWM_INPUT_1,
			.captureInputMode = 0x03,
			.captureInput = CY_TCPWM_INPUT_0,
		};

		Cy_TCPWM_Counter_Init(base, chd->cnt_idx, &cnt_cfg);
		Cy_TCPWM_Counter_Enable(base, chd->cnt_idx);
		Cy_TCPWM_TriggerStart_Single(base, chd->cnt_idx);

		enable_sys_int(cc->irq_num, cc->irq_priority,
			       tgpio_ifx_ch_isr, chd);
	}

	Cy_SysClk_PeriPclkEnableDivider(cfg->clock_peri_group,
					 cfg->divider_type,
					 cfg->divider_sel);

	return 0;
}

#define TGPIO_CH_PINCTRL_DEFINE(child) PINCTRL_DT_DEFINE(child);

#define TGPIO_CH_CFG(child)                                                \
	{                                                                  \
		.channel_idx = DT_REG_ADDR(child),                         \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(child),                  \
		.irq_num = DT_PROP_BY_IDX(child, system_interrupts, 0),   \
		.irq_priority = DT_PROP_BY_IDX(child, system_interrupts, 1), \
	},

#define TGPIO_COUNT(child) + 1
#define TGPIO_N(n) (0 DT_INST_FOREACH_CHILD_STATUS_OKAY(n, TGPIO_COUNT))

/* DT: tcpwm0 (grandparent) -> grpX (parent) -> tgpio (inst) -> channels */
#define TGPIO_IFX_DEFINE(n)                                                \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(n, TGPIO_CH_PINCTRL_DEFINE)     \
                                                                           \
	static const struct tgpio_ifx_ch_config                            \
		tgpio_ch_cfg_##n[] = {                                     \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(n, TGPIO_CH_CFG)        \
	};                                                                 \
                                                                           \
	static struct tgpio_ifx_ch_data tgpio_chd_##n[TGPIO_N(n)];        \
                                                                           \
	static const struct tgpio_ifx_config tgpio_cfg_##n = {             \
		.base = DT_REG_ADDR(DT_PARENT(DT_INST_PARENT(n))),        \
		.grp_base = DT_INST_REG_ADDR(n),                           \
		.num_channels = TGPIO_N(n),                                \
		.ch_cfg = tgpio_ch_cfg_##n,                                \
		.clock_peri_group =                                        \
			DT_INST_PROP(n,             \
				ifx_peri_group),                           \
		.divider_type = DT_INST_PROP(n, divider_type),             \
		.divider_sel = DT_INST_PROP(n, divider_sel),               \
		.divider_val = DT_INST_PROP(n, divider_val),               \
	};                                                                 \
                                                                           \
	DEVICE_DT_INST_DEFINE(n, tgpio_ifx_init, NULL,                    \
			      tgpio_chd_##n, &tgpio_cfg_##n,               \
			      POST_KERNEL,                                 \
			      CONFIG_TIMEAWARE_GPIO_INIT_PRIORITY,         \
			      &tgpio_ifx_api);

DT_INST_FOREACH_STATUS_OKAY(TGPIO_IFX_DEFINE)
