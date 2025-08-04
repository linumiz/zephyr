/*
 * Copyright (c) 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_dac

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

/* TI Driverlib includes */
#include <ti/devices/msp/peripherals/hw_dac12.h>
#include <ti/driverlib/dl_dac12.h>

/* 12-bit DAC Binary Representation Range */
#define DAC12_BINARY_MAX 4095
#define DAC12_BINARY_MIN 0

/* 12-bit DAC Two's Complement Representation Range */
#define DAC12_TWOS_COMP_MAX 2047
#define DAC12_TWOS_COMP_MIN (-2048)

/* 8-bit DAC Binary Representation Range */
#define DAC8_BINARY_MAX 255
#define DAC8_BINARY_MIN 0

/* 8-bit DAC Two's Complement Representation Range */
#define DAC8_TWOS_COMP_MAX 127
#define DAC8_TWOS_COMP_MIN (-128)

/* Timeout to wait until the DAC module is ready after enabling */
#define DAC_MOD_RDY_TIMEOUT K_MSEC(CONFIG_DAC_MSPM0_TIMEOUT_MS)

struct dac_mspm0_config {
	DAC12_Regs *dac_base;
	DL_DAC12_REPRESENTATION dac_repr;
	void (*irq_config_func)(const struct device *dev);
};

struct dac_mspm0_data {
	uint8_t resolution;
	struct k_sem mod_rdy;
};

static void dac_mspm0_isr(const struct device *dev)
{
	const struct dac_mspm0_config *config = dev->config;
	struct dac_mspm0_data *data = dev->data;
	DAC12_Regs *dac12 = config->dac_base;

	if (DL_DAC12_getPendingInterrupt(dac12) == DL_DAC12_IIDX_MODULE_READY) {
		k_sem_give(&data->mod_rdy);
	}
}

static int dac_mspm0_channel_setup(const struct device *dev,
				   const struct dac_channel_cfg *channel_cfg)
{
	const struct dac_mspm0_config *config = dev->config;
	struct dac_mspm0_data *data = dev->data;
	DAC12_Regs *dac12 = config->dac_base;

	if (channel_cfg->channel_id != 0) {
		return -EINVAL;
	}

	if (channel_cfg->resolution != 8 && channel_cfg->resolution != 12) {
		return -ENOTSUP;
	}

	k_sem_reset(&data->mod_rdy);

	/* Ensure DAC is disabled before configuration */
	if (DL_DAC12_isEnabled(dac12)) {
		DL_DAC12_disable(dac12);
	}

	DL_DAC12_configDataFormat(dac12, config->dac_repr,
				  (channel_cfg->resolution == 12) ? DL_DAC12_RESOLUTION_12BIT
								  : DL_DAC12_RESOLUTION_8BIT);

	/* buffered must be true to enable amplifier for output drive */
	DL_DAC12_setAmplifier(dac12,
			      (channel_cfg->buffered) ? DL_DAC12_AMP_ON : DL_DAC12_AMP_OFF_0V);

	DL_DAC12_setReferenceVoltageSource(dac12, DL_DAC12_VREF_SOURCE_VDDA_VSSA);

	/* internal must be true to route output to OPA, ADC, COMP and DAC_OUT pin */
	if (channel_cfg->internal) {
		DL_DAC12_enableOutputPin(dac12);
	} else {
		DL_DAC12_disableOutputPin(dac12);
	}

	DL_DAC12_enable(dac12);
	if (!DL_DAC12_isEnabled(dac12)) {
		return -EAGAIN;
	}

	if (k_sem_take(&data->mod_rdy, DAC_MOD_RDY_TIMEOUT) != 0) {
		return -ETIMEDOUT;
	}

	data->resolution = channel_cfg->resolution;
	DL_DAC12_performSelfCalibrationBlocking(dac12);

	return 0;
}

static int dac_mspm0_write_value(const struct device *dev, uint8_t channel, uint32_t value)
{
	const struct dac_mspm0_config *config = dev->config;
	struct dac_mspm0_data *data = dev->data;
	DAC12_Regs *dac12 = config->dac_base;

	if (!DL_DAC12_isEnabled(dac12)) {
		return -EAGAIN;
	}

	if (channel != 0) {
		return -EINVAL;
	}

	if (data->resolution == 12) {
		if ((config->dac_repr == DL_DAC12_REPRESENTATION_BINARY) &&
		    (value > DAC12_BINARY_MAX || value < DAC12_BINARY_MIN)) {
			return -EINVAL;
		}

		if ((config->dac_repr == DL_DAC12_REPRESENTATION_TWOS_COMPLEMENT) &&
		    ((int32_t)value > DAC12_TWOS_COMP_MAX ||
		     (int32_t)value < DAC12_TWOS_COMP_MIN)) {
			return -EINVAL;
		}

		DL_DAC12_output12(dac12, value);

	} else if (data->resolution == 8) {
		if ((config->dac_repr == DL_DAC12_REPRESENTATION_BINARY) &&
		    (value > DAC8_BINARY_MAX || value < DAC8_BINARY_MIN)) {
			return -EINVAL;
		}

		if ((config->dac_repr == DL_DAC12_REPRESENTATION_TWOS_COMPLEMENT) &&
		    ((int32_t)value > DAC8_TWOS_COMP_MAX || (int32_t)value < DAC8_TWOS_COMP_MIN)) {
			return -EINVAL;
		}

		DL_DAC12_output8(dac12, (uint8_t)value);

	} else {
		/* Setup must be done with valid resolution */
		return -EINVAL;
	}

	return 0;
}

static int dac_mspm0_init(const struct device *dev)
{
	const struct dac_mspm0_config *config = dev->config;
	struct dac_mspm0_data *data = dev->data;
	DAC12_Regs *dac12 = config->dac_base;

	DL_DAC12_enablePower(dac12);
	if (!DL_DAC12_isPowerEnabled(dac12)) {
		return -EAGAIN;
	}

	DL_DAC12_disableInterrupt(dac12, DL_DAC12_INTERRUPT_MODULE_READY);
	DL_DAC12_clearInterruptStatus(dac12, DL_DAC12_INTERRUPT_MODULE_READY);

	config->irq_config_func(dev);
	DL_DAC12_enableInterrupt(dac12, DL_DAC12_INTERRUPT_MODULE_READY);

	return 0;
}

static DEVICE_API(dac, dac_mspm0_driver_api) = {
	.channel_setup = dac_mspm0_channel_setup,
	.write_value   = dac_mspm0_write_value
};

#define DAC_MSPM0_DEFINE(id)									\
												\
	static void dac_mspm0_irq_config_##id(const struct device *dev)				\
	{											\
		IRQ_CONNECT(DT_INST_IRQN(id), DT_INST_IRQ(id, priority), dac_mspm0_isr,		\
				DEVICE_DT_INST_GET(id), 0);					\
		irq_enable(DT_INST_IRQN(id));							\
	};											\
												\
	static const struct dac_mspm0_config dac_mspm0_config_##id = {				\
		.dac_base = (DAC12_Regs *)DT_INST_REG_ADDR(id),					\
												\
		.dac_repr = IS_ENABLED(CONFIG_DAC_MSPM0_DFM_TWOS_COMPLEMENT)			\
					? DL_DAC12_REPRESENTATION_TWOS_COMPLEMENT		\
					: DL_DAC12_REPRESENTATION_BINARY,			\
		.irq_config_func = dac_mspm0_irq_config_##id,					\
	};											\
												\
	static struct dac_mspm0_data dac_mspm0_data_##id = {					\
		/* Config at channel setup */							\
		.resolution = 0,								\
		.mod_rdy = Z_SEM_INITIALIZER(dac_mspm0_data_##id.mod_rdy, 0, 1),		\
	};											\
												\
	DEVICE_DT_INST_DEFINE(id, &dac_mspm0_init, NULL, &dac_mspm0_data_##id,			\
			      &dac_mspm0_config_##id, POST_KERNEL, CONFIG_DAC_INIT_PRIORITY,	\
			      &dac_mspm0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAC_MSPM0_DEFINE)
