/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <soc.h>

#include "gpio_tc4xx.h"

#define DT_DRV_COMPAT infineon_tc4xx_gpio

/**
 * @brief Common gpio flags to custom flags
 */
static int gpio_tc4xx_flags_to_drvcfg(gpio_flags_t flags, Ifx_P_PADCFG_DRVCFG *drvcfg)
{
	bool is_input = flags & GPIO_INPUT;
	bool is_output = flags & GPIO_OUTPUT;

	/* Disconnect not supported */
	if (!is_input && !is_output) {
		return -ENOTSUP;
	}

	/* Open source not supported*/
	if (flags & GPIO_OPEN_SOURCE) {
		return -ENOTSUP;
	}

	/* Pull up & pull down not supported in output mode */
	if (is_output && (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
		return -ENOTSUP;
	}

	drvcfg->B.PL = 0;

	if (is_input) {
		drvcfg->B.DIR = 0;
		if (flags & (GPIO_PULL_UP)) {
			drvcfg->B.MODE = GPIO_TC4XX_INPUT_PULL_UP;
		} else if (flags & (GPIO_PULL_DOWN)) {
			drvcfg->B.MODE = GPIO_TC4XX_INPUT_PULL_DOWN;
		} else {
			drvcfg->B.MODE = GPIO_TC4XX_INPUT_GPIO;
		}
	}
	if (is_output) {
		drvcfg->B.DIR = 1;
		drvcfg->B.MODE = 0;
		drvcfg->B.PD = 0;
		if (flags & GPIO_OPEN_DRAIN) {
			drvcfg->B.OD = 1;
		} else {
			drvcfg->B.OD = 0;
		}
	}

	return 0;
}

#if defined(CONFIG_GPIO_GET_CONFIG)
static int gpio_tc4xx_pincfg_to_flags(uint32_t iocr, uint32_t out, gpio_flags_t *out_flags)
{
	if (iocr & TC3XX_IOCR_OUTPUT) {
		if (out) {
			*out_flags = GPIO_OUTPUT_HIGH;
		} else {
			*out_flags = GPIO_OUTPUT_LOW;
		}
		if (iocr & TC3XX_IOCR_OPEN_DRAIN) {
			*out_flags |= GPIO_OPEN_DRAIN;
		}
	} else {
		*out_flags = GPIO_INPUT;
		if (iocr & TC3XX_IOCR_PULL_DOWN) {
			*out_flags |= GPIO_PULL_DOWN;
		} else if (iocr & TC3XX_IOCR_PULL_UP) {
			*out_flags |= GPIO_PULL_UP;
		}
	}

	return 0;
}
#endif

static int gpio_tc4xx_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_tc4xx_config *cfg = dev->config;

	*value = cfg->base->OUT.U;

	return 0;
}

static int gpio_tc4xx_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct gpio_tc4xx_config *cfg = dev->config;
	uint32_t clear, set;
	mask &= 0xFFFF;
	value &= 0xFFFF;

	set = (mask & value);
	clear = (mask & ~value);

	cfg->base->OMR.U = (clear << 16) | set;

	return 0;
}

static int gpio_tc4xx_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_tc4xx_config *cfg = dev->config;

	cfg->base->OMSR.U = 0xFFFF & pins;

	return 0;
}

static int gpio_tc4xx_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_tc4xx_config *cfg = dev->config;

	cfg->base->OMCR.U = (0xFFFF & pins) << 16;

	return 0;
}

static int gpio_tc4xx_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_tc4xx_config *cfg = dev->config;
	uint32_t out;
	uint64_t swap;

	do {
		out = cfg->base->OUT.U;
		swap = ((uint64_t)out << 32) | (out ^ pins);
		__asm("	cmpswap.w [%1]+0, %A0\n" : "+d"(swap) : "a"(&cfg->base->OUT));
	} while ((swap & 0xFFFFFFFF) != out);

	return 0;
}

/**
 * @brief Configure pin or port
 */
static int gpio_tc4xx_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_tc4xx_config *cfg = dev->config;
	int err;
	Ifx_P_PADCFG_DRVCFG drvcfg;

	/* figure out if we can map the requested GPIO
	 * configuration
	 */
	err = gpio_tc4xx_flags_to_drvcfg(flags, &drvcfg);
	if (err != 0) {
		return err;
	}

	if ((flags & GPIO_OUTPUT) != 0) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			gpio_tc4xx_port_set_bits_raw(dev, BIT(pin));
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			gpio_tc4xx_port_clear_bits_raw(dev, BIT(pin));
		}
	}

	volatile void *accrgp_prote = &cfg->base->ACCGRP[cfg->access_group].PROTE;
	bool init = aurix_prot_get_state(accrgp_prote) == AURIX_PROT_STATE_INIT;
	if (!init) {
		aurix_prot_set_state(accrgp_prote, AURIX_PROT_STATE_CONFIG);
	}
	cfg->base->PADCFG[pin].DRVCFG = drvcfg;
	if (!init) {
		aurix_prot_set_state(accrgp_prote, AURIX_PROT_STATE_RUN);
	}

	return 0;
}

#if defined(CONFIG_GPIO_GET_CONFIG)
/**
 * @brief Get configuration of pin
 */
static int gpio_tc4xx_get_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t *flags)
{
	const struct gpio_tc4xx_config *cfg = dev->config;

	gpio_tc4xx_pincfg_to_flags(*(cfg->base + TC3XX_IOCR_OFFSET / 4),
				   *(cfg->base TC3XX_OUT_OFFSET), flags);

	return 0;
}
#endif

struct gpio_tc4xx_irq_tim {
	uint8_t tim;
	uint8_t ch;
	uint8_t mux;
};

void gpio_tc4xx_irq_tim_ch(uint8_t port, uint8_t pin, struct gpio_tc4xx_irq_tim *tims,
			   uint8_t *tim_count);
#include "IfxEgtm_reg.h"

static void gpio_tc4_isr_pin(struct device *dev)
{
	const struct gpio_tc4xx_config *cfg = dev->config;
	struct gpio_tc4xx_data *data = dev->data;
	uint16_t pins = 0;
	uint16_t i;
	for (i = 0; i < cfg->irq_source_count; i++) {
		Ifx_EGTM_CLS_TIM_CH_IRQ_NOTIFY notify = MODULE_EGTM.CLS[cfg->irq_sources[i].cls]
								.TIM.CH[cfg->irq_sources[i].ch]
								.IRQ_NOTIFY;
		if (notify.B.NEWVAL) {
			pins |= (1 << cfg->irq_sources[i].pin);
			MODULE_EGTM.CLS[cfg->irq_sources[i].cls]
				.TIM.CH[cfg->irq_sources[i].ch]
				.IRQ_NOTIFY = notify;
		}
	}
	gpio_fire_callbacks(&data->callbacks, dev, pins);
}

static int gpio_tc4xx_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					      enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_tc4xx_config *cfg = dev->config;
	const struct gpio_tc4xx_irq_source *irq_src = NULL;
	uint32_t i = 0;

	for (i = 0; i < cfg->irq_source_count; i++) {
		if (cfg->irq_sources[i].pin == pin) {
			irq_src = &cfg->irq_sources[i];
		}
	}

	if (irq_src == NULL) {
		return -ENOTSUP;
	}

	if (irq_src->type == TC4XX_IRQ_TYPE_GTM) {
		if (MODULE_EGTM.CLC.B.DISS == 1) {
			return -EIO;
		}

		Ifx_EGTM_CLS_TIM_CH_CTRL ctrl = {.U = 0};
		Ifx_EGTM_CLS_TIM_CH_IRQ_EN irq_en = {.B.NEWVAL_IRQ_EN = 1};
		ctrl.B.TIM_EN = 1;
		ctrl.B.CLK_SEL = 7;
		ctrl.B.FLT_CNT_FRQ = 0x3;
		ctrl.B.TIM_MODE = mode == GPIO_INT_MODE_EDGE ? 0x2 : 0x5;
		ctrl.B.DSL = trig == GPIO_INT_TRIG_HIGH ? 1 : 0;
		ctrl.B.ISL = trig == GPIO_INT_TRIG_BOTH ? 1 : 0;
		ctrl.B.FLT_EN = 1;
		ctrl.B.FLT_MODE_FE = mode == GPIO_INT_MODE_LEVEL ? 0x1 : 0;
		ctrl.B.FLT_MODE_RE = mode == GPIO_INT_MODE_LEVEL ? 0x1 : 0;

		MODULE_EGTM.TIMINSEL[irq_src->cls].U =
			(MODULE_EGTM.TIMINSEL[irq_src->cls].U & ~(0xF << (4 * irq_src->ch))) |
			(irq_src->mux << (4 * irq_src->ch));
		MODULE_EGTM.CLS[irq_src->cls].TIM.CH[irq_src->ch].CTRL = ctrl;
		MODULE_EGTM.CLS[irq_src->cls].TIM.CH[irq_src->ch].CNTS.U = 10;
		MODULE_EGTM.CLS[irq_src->cls].TIM.CH[irq_src->ch].FLT_FE.U = 0x5;
		MODULE_EGTM.CLS[irq_src->cls].TIM.CH[irq_src->ch].FLT_RE.U = 0x5;
		MODULE_EGTM.CLS[irq_src->cls].TIM.CH[irq_src->ch].IRQ_EN = irq_en;

		irq_enable((0x1E60 / 4 + irq_src->cls * 8 + irq_src->ch));

		return 0;
	}

	return -ENOTSUP;
}

static int gpio_tc4xx_manage_callback(const struct device *dev, struct gpio_callback *callback,
				      bool set)
{
	struct gpio_tc4xx_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static DEVICE_API(gpio, tc4xx_gpio_driver_api) = {
	.pin_configure = gpio_tc4xx_config,
#if defined(CONFIG_GPIO_GET_CONFIG)
	.pin_get_config = gpio_tc4xx_get_config,
#endif /* CONFIG_GPIO_GET_CONFIG */
	.port_get_raw = gpio_tc4xx_port_get_raw,
	.port_set_masked_raw = gpio_tc4xx_port_set_masked_raw,
	.port_set_bits_raw = gpio_tc4xx_port_set_bits_raw,
	.port_clear_bits_raw = gpio_tc4xx_port_clear_bits_raw,
	.port_toggle_bits = gpio_tc4xx_port_toggle_bits,
	.pin_interrupt_configure = gpio_tc4xx_pin_interrupt_configure,
	.manage_callback = gpio_tc4xx_manage_callback,
};

/**
 * @brief Initialize GPIO port
 *
 * Perform basic initialization of a GPIO port. The code will
 * enable the clock for corresponding peripheral.
 *
 * @param dev GPIO device struct
 *
 * @return 0
 */
static int gpio_tc4xx_init(const struct device *dev)
{
	const struct gpio_tc4xx_config *cfg = dev->config;

	cfg->config_func(dev);

	return 0;
}

#define GPIO_TC4XX_IRQ_CONNECT(node, prop, i)                                                      \
	IRQ_CONNECT((0x1E60 / 4 + 8 * ((DT_PROP_BY_IDX(node, prop, i) >> 8) & 0xF) +               \
		     ((DT_PROP_BY_IDX(node, prop, i) >> 4) & 0xF)),                                \
		    10, gpio_tc4_isr_pin, DEVICE_DT_GET(node), 0);
#define GPIO_TC4XX_CONFIG_FUNC(n)                                                                    \
	static void gpio_tc4xx_config_func_##n(const struct device *dev)                             \
	{                                                                                            \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(n, irq_sources),                                 \
			    (DT_INST_FOREACH_PROP_ELEM(n, irq_sources, GPIO_TC4XX_IRQ_CONNECT)),   \
			    ()) \
	}

#define GPIO_TC4XX_IRQ_SOURCE(node_id, prop, i)                                                    \
	{(DT_PROP_BY_IDX(node_id, prop, i) & 0xF),                                                 \
	 ((DT_PROP_BY_IDX(node_id, prop, i) >> 4) & 0xF),                                          \
	 ((DT_PROP_BY_IDX(node_id, prop, i) >> 8) & 0xF),                                          \
	 ((DT_PROP_BY_IDX(node_id, prop, i) >> 12) & 0x3),                                         \
	 ((DT_PROP_BY_IDX(node_id, prop, i) >> 28) & 0xF)}
#define GPIO_TC4XX_IRQ_SOURCE_IS_GTM(node_id, prop, id)                                            \
	(((DT_PROP_BY_IDX(node_id, prop, id) >> 12) & 0x3) == TC4XX_IRQ_TYPE_GTM)
#define GPIO_TC4XX_IRQ_SOURCE_IS_ERU(node_id, prop, id)                                            \
	(((DT_PROP_BY_IDX(node_id, prop, id) >> 12) & 0x3) == TC4XX_IRQ_TYPE_ERU)
#define GPIO_TC4XX_ENABLE_GTM(n)                                                                   \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, irq_sources), \
			 (DT_INST_FOREACH_PROP_ELEM_SEP(n, irq_sources, GPIO_TC4XX_IRQ_SOURCE_IS_GTM, (||))), \
			 (false))
#define GPIO_TC4XX_ENABLE_ERU(n)                                                                   \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, irq_sources), \
			 (DT_INST_FOREACH_PROP_ELEM_SEP(n, irq_sources, GPIO_TC4XX_IRQ_SOURCE_IS_ERU, (||))), \
			 (false))
#define GPIO_TC4XX_INIT(n)                                                                         \
	GPIO_TC4XX_CONFIG_FUNC(n)                                                                  \
	static const struct gpio_tc4xx_irq_source gpio_tc4xx_irq_sources_##n[] = {COND_CODE_1(     \
		DT_INST_NODE_HAS_PROP(n, irq_sources),                                             \
		(DT_INST_FOREACH_PROP_ELEM_SEP(n, irq_sources, GPIO_TC4XX_IRQ_SOURCE, (, ))),      \
		())};      \
	static const struct gpio_tc4xx_config gpio_tc4xx_config_##n = {                            \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),               \
			},                                                                         \
		.base = (Ifx_P *)DT_INST_REG_ADDR(n),                                              \
		.irq_source_count = DT_INST_PROP_LEN_OR(n, irq_sources, 0),                        \
		.irq_sources = gpio_tc4xx_irq_sources_##n,                                         \
		.config_func = gpio_tc4xx_config_func_##n,                                         \
		.enable_gtm = GPIO_TC4XX_ENABLE_GTM(n),                                            \
		.enable_eru = GPIO_TC4XX_ENABLE_ERU(n),                                            \
	};                                                                                         \
                                                                                                   \
	static struct gpio_tc4xx_data gpio_tc4xx_data_##n = {};                                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, gpio_tc4xx_init, NULL, &gpio_tc4xx_data_##n,                      \
			      &gpio_tc4xx_config_##n, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,     \
			      &tc4xx_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_TC4XX_INIT)
