/*
 * Copyright (c) 2025 Texas Instruments
 * Copyright (c) 2025 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_dac

#include <zephyr/kernel.h>
#include <zephyr/drivers/dac.h>

/* TI Driverlib includes */
#include <ti/driverlib/dl_dac12.h>
#include <ti/devices/msp/peripherals/hw_dac12.h>

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

struct dac_mspm0_config {
	DAC12_Regs *dac_base;
	DL_DAC12_REPRESENTATION dac_repr;
	DL_DAC12_VREF_SOURCE dac_vref_src;
};

struct dac_mspm0_data {
	uint8_t resolution;
	bool setup_done;
};

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

	data->resolution = channel_cfg->resolution;

	/* Ensure DAC is disabled before configuration */
	if (DL_DAC12_isEnabled(dac12)) {
		DL_DAC12_disable(dac12);
	}

	DL_DAC12_configDataFormat(dac12, config->dac_repr,
			(channel_cfg->resolution == 12) ?
			DL_DAC12_RESOLUTION_12BIT : DL_DAC12_RESOLUTION_8BIT);

	DL_DAC12_setAmplifier(dac12,
			(channel_cfg->buffered) ?
			DL_DAC12_AMP_ON : DL_DAC12_AMP_OFF_0V);

	DL_DAC12_setReferenceVoltageSource(dac12, config->dac_vref_src);

	if (channel_cfg->internal) {
		DL_DAC12_enableOutputPin(dac12);
	} else {
		DL_DAC12_disableOutputPin(dac12);
	}

	DL_DAC12_enable(dac12);
	if (!DL_DAC12_isEnabled(dac12)) {
		return -EAGAIN;
	}

	DL_DAC12_performSelfCalibrationBlocking(dac12);
	data->setup_done = true;

	return 0;
}

static int dac_mspm0_write_value(const struct device *dev,
		uint8_t channel, uint32_t value)
{
	const struct dac_mspm0_config *config = dev->config;
	struct dac_mspm0_data *data = dev->data;
	DAC12_Regs *dac12 = config->dac_base;

	if (!(data->setup_done)) {
		return -EIO;
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
	} else {
		if ((config->dac_repr == DL_DAC12_REPRESENTATION_BINARY) &&
				(value > DAC8_BINARY_MAX || value < DAC8_BINARY_MIN)) {
			return -EINVAL;
		}

		if ((config->dac_repr == DL_DAC12_REPRESENTATION_TWOS_COMPLEMENT) &&
				((int32_t)value > DAC8_TWOS_COMP_MAX ||
				 (int32_t)value < DAC8_TWOS_COMP_MIN)) {
			return -EINVAL;
		}

		DL_DAC12_output8(dac12, (uint8_t)value);
	}

	return 0;
}

static int dac_mspm0_init(const struct device *dev)
{
	const struct dac_mspm0_config *config = dev->config;
	DAC12_Regs *dac = config->dac_base;

	DL_DAC12_reset(dac);

	DL_DAC12_enablePower(dac);
	if (!DL_DAC12_isPowerEnabled(dac)) {
		return -EIO;
	}

	return 0;
}

static DEVICE_API(dac, dac_mspm0_driver_api) = {
	.channel_setup = dac_mspm0_channel_setup,
	.write_value   = dac_mspm0_write_value
};

static const struct dac_mspm0_config dac_mspm0_config = {
	.dac_base = (DAC12_Regs *)DT_INST_REG_ADDR(0),

	.dac_repr = IS_ENABLED(CONFIG_DAC_MSPM0_DFM_TWOS_COMPLEMENT) ?
		DL_DAC12_REPRESENTATION_TWOS_COMPLEMENT : DL_DAC12_REPRESENTATION_BINARY,

	.dac_vref_src = IS_ENABLED(CONFIG_DAC_MSPM0_VREF_SOURCE_VEREF) ?
		DL_DAC12_VREF_SOURCE_VEREFP_VEREFN : DL_DAC12_VREF_SOURCE_VDDA_VSSA,
};

static struct dac_mspm0_data dac_mspm0_data = {
	.resolution = 12,
	.setup_done = false,
};

DEVICE_DT_INST_DEFINE(0,
		&dac_mspm0_init,
		NULL,
		&dac_mspm0_data,
		&dac_mspm0_config,
		POST_KERNEL,
		CONFIG_DAC_INIT_PRIORITY,
		&dac_mspm0_driver_api);
