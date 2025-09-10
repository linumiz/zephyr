/*
 * Copyright 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_bq25620_bc

#include "bc12_bq25620.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mfd/mfd_bq2562x.h>
#include <zephyr/drivers/usb/usb_bc12.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(BQ25620_BC, CONFIG_USB_BC12_LOG_LEVEL);

struct bc12_bq25620_config {
	const struct device *mfd;
	enum bc12_type charging_mode;
};

struct bc12_bq25620_data {
	struct bc12_partner_state partner_state;
	struct bq2562x_mfd_callback bq25620_cb;
	bc12_callback_t result_cb;
	void *result_cb_data;
};

static const enum bq25620_mode charging_mode_to_host_mode[] = {
	[BC12_TYPE_NONE] = BQ2562X_POWER_DOWN,
	[BC12_TYPE_SDP] = BQ2562X_SDP_HOST_MODE,
	[BC12_TYPE_CDP] = BQ2562X_CDP_HOST_MODE,
	[BC12_TYPE_DCP] = BQ2562X_DCP_HOST_MODE,
};

static void bq25620_notify_callback(const struct device *dev,
				    struct bc12_partner_state *const state)
{
	struct bc12_bq25620_data *data = dev->data;

	if (data->result_cb != NULL) {
		data->result_cb(dev, state, data->result_cb_data);
	}
}

static void bq25620_update_charging_partner(const struct device *dev,
					    struct bc12_partner_state *const state)
{
	struct bc12_bq25620_data *data = dev->data;

	if (state) {
		/* Now update callback with the new partner type */
		data->partner_state = *state;
		bq25620_notify_callback(dev, state);
	} else {
		data->partner_state.bc12_role = BC12_DISCONNECTED;
		bq25620_notify_callback(dev, NULL);
	}
}

static int bq25620_get_vbus_adc(const struct device *dev, uint32_t *volt)
{
	const struct bc12_bq25620_config *config = dev->config;
	uint8_t vbus_adc[2] = {0};
	int ret;

	ret = mfd_bq2562x_burst_read_dt(config->mfd, BQ2562X_ADC_VBUS_LSB, vbus_adc,
					ARRAY_SIZE(vbus_adc));
	if (ret) {
		LOG_ERR("Failed to read the VBUS reg ,ret :%d", ret);
		return ret;
	}

	*volt = ((vbus_adc[1] << 8) | vbus_adc[0]) >> BQ2562X_ADC_VBUS_SHIFT;
	*volt = *volt * BQ2562X_ADC_VBUS_STEP_UV;

	return 0;

}

static int bq25620_get_ibus_adc(const struct device *dev, int32_t *ibus)
{
	const struct bc12_bq25620_config *config = dev->config;
	uint8_t ibus_adc[2] = {0};
	uint16_t temp;
	int ret;

	ret = mfd_bq2562x_burst_read_dt(config->mfd, BQ2562X_ADC_IBUS_LSB, ibus_adc,
					ARRAY_SIZE(ibus_adc));
	if (ret) {
		LOG_ERR("Failed to read the IBUS reg, ret : %d", ret);
		return ret;
	}

	temp = sys_get_le16(ibus_adc);
	if (temp & BIT(15)) {
		temp = ~temp + 1;
		*ibus = (temp >> BQ2562X_ADC_IBUS_SHIFT) * BQ2562X_ADC_CURR_STEP_UA * -1;
	} else {
		*ibus = (temp >> BQ2562X_ADC_IBUS_SHIFT) * BQ2562X_ADC_CURR_STEP_UA;
	}

	return 0;
}

static int bq25620_set_iotg(const struct device *dev, uint32_t iotg_ua)
{
	const struct bc12_bq25620_config *config = dev->config;
	uint8_t curr[2] = {0};

	iotg_ua = CLAMP(iotg_ua, BQ2562X_IOTG_MIN, BQ2562X_IOTG_MAX);

	iotg_ua = (iotg_ua / BQ2562X_IOTG_STEP_UA) << BQ2562X_IOTG_SHIFT;
	curr[1] = (iotg_ua >> 8) & BQ2562X_IOTG_MSB_MSK;
	curr[0] = iotg_ua & BQ2562X_IOTG_LSB_MSK;

	return mfd_bq2562x_burst_write_dt(config->mfd, BQ2562X_IOTG_LSB, curr, ARRAY_SIZE(curr));
}

static int bq25620_set_votg(const struct device *dev, uint32_t votg_uv)
{
	const struct bc12_bq25620_config *config = dev->config;
	uint8_t volt[2] = {0};

	votg_uv = CLAMP(votg_uv, BQ2562X_VOTG_MIN, BQ2562X_VOTG_MAX);

	votg_uv = (votg_uv / BQ2562X_VOTG_STEP_UV) << BQ2562X_VOTG_SHIFT;
	volt[1] = (votg_uv >> 8) & BQ2562X_VOTG_MSB_MSK;
	volt[0] = votg_uv & BQ2562X_VOTG_LSB_MSK;

	return mfd_bq2562x_burst_write_dt(config->mfd, BQ2562X_VOTG_LSB, volt, ARRAY_SIZE(volt));
}

static int bq25620_get_pd_type(const struct device *dev, struct bc12_partner_state *partner_state)
{
	const struct bc12_bq25620_config *config = dev->config;
	uint8_t val;
	int ret;

	ret = mfd_bq2562x_reg_read_byte_dt(config->mfd, BQ2562X_CHRG_STAT_1, &val);
	if (ret) {
		LOG_ERR("Failed to Read CHRG_STAT_1 Reg: %d", ret);
		return ret;
	}

	val = FIELD_GET(BQ2562X_VBUS_STAT_MSK, val);

	switch (val) {
	case BQ2562X_USB_SDP:
		partner_state->type = BC12_TYPE_SDP;
		break;
	case BQ2562X_USB_CDP:
		partner_state->type = BC12_TYPE_CDP;
		break;
	case BQ2562X_HVDCP:
		__fallthrough;
	case BQ2562X_USB_DCP:
		partner_state->type = BC12_TYPE_DCP;
		break;
	case BQ2562X_UNKNOWN_500MA:
		partner_state->type = BC12_TYPE_UNKNOWN;
		break;
	case BQ2562X_NON_STANDARD:
		partner_state->type = BC12_TYPE_PROPRIETARY;
		break;
	case BQ2562X_OTG_MODE:
		partner_state->pd_partner_connected = true;
		partner_state->bc12_role = BC12_CHARGING_PORT;
		break;
	default:
		partner_state->bc12_role = BC12_DISCONNECTED;
		break;
	}
	return ret;
}

static void bq25620_int_work_handler(const struct device *dev)
{
	const struct bc12_bq25620_config *config = dev->config;
	struct bc12_partner_state partner_state = {0};
	uint32_t vbus_uv;
	int32_t ibus_ua;
	uint8_t val;
	int ret;

	ret = mfd_bq2562x_reg_read_byte_dt(config->mfd, BQ2562X_CHRG_FLAG_1, &val);
	if (ret) {
		LOG_ERR("Failed to Read CHRG_FLAG_1 Reg: %d", ret);
		return;
	}

	if (val & BQ2562X_VBUS_FLAG_MSK) {
		partner_state.pd_partner_connected = false;
		partner_state.bc12_role = BC12_PORTABLE_DEVICE;

		ret = bq25620_get_pd_type(dev, &partner_state);
		if (ret) {
			LOG_ERR("Failed to get pd type ret: %d", ret);
			return;
		}

		if (partner_state.bc12_role == BC12_DISCONNECTED) {
			bq25620_update_charging_partner(dev, NULL);
			return;
		} else if (partner_state.bc12_role == BC12_PORTABLE_DEVICE){

			ret = bq25620_get_ibus_adc(dev, &ibus_ua);
			if (ret) {
				LOG_ERR("Failed to read the IBUS ADC ret: %d", ret);
				return;
			}

			ret = bq25620_get_vbus_adc(dev, &vbus_uv);
			if (ret) {
				LOG_ERR("Failed to read the VBUS ADC ret: %d", ret);
				return;
			}

			partner_state.current_ua = ibus_ua;
			partner_state.voltage_uv = (int)vbus_uv;
		}
		bq25620_update_charging_partner(dev, &partner_state);
	}
}

static int bq25620_disconnect(const struct device *dev)
{
	const struct bc12_bq25620_config *config = dev->config;
	int ret;

	/* Disable interrupts during mode change */
	mfd_bq2562x_enable_interrupt_pin(config->mfd, false);

	/* Disable auto indent to stop detection in dpdm pins */
	ret = mfd_bq2562x_reg_update_byte_dt(config->mfd, BQ2562X_TIMER_CTRL,
					     BQ2562X_TIMER_AUTO_INDET, 0);
	if (ret) {
		goto cleanup;
	}

	/* Enable HIZ mode to put VBUS in high impedance */
	ret = mfd_bq2562x_reg_update_byte_dt(config->mfd, BQ2562X_CHRG_CTRL_1,
					     BQ2562X_CHRG_HIZ, BQ2562X_CHRG_HIZ);
	if (ret) {
		goto cleanup;
	}

	bq25620_update_charging_partner(dev, NULL);

cleanup:
	mfd_bq2562x_enable_interrupt_pin(config->mfd, true);
	return ret;
}

static int bq25620_set_portable_device(const struct device *dev)
{
	const struct bc12_bq25620_config *config = dev->config;
	struct bc12_bq25620_data *data = dev->data;
	struct bc12_partner_state partner_state;
	uint32_t vbus_uv;
	int32_t ibus_ua;
	int ret;

	if (data->partner_state.bc12_role == BC12_PORTABLE_DEVICE) {
		/* Device is already in the same mode */
		return 0;
	}

	/* Disable interrupts during mode change */
	mfd_bq2562x_enable_interrupt_pin(config->mfd, false);

	/* Enable auto indet to start dpdm detection */
	ret = mfd_bq2562x_reg_update_byte_dt(config->mfd, BQ2562X_TIMER_CTRL,
					     BQ2562X_TIMER_AUTO_INDET, BQ2562X_TIMER_AUTO_INDET);
	if (ret) {
		goto cleanup;
	}

	/* Disable HIZ mode */
	ret = mfd_bq2562x_reg_update_byte_dt(config->mfd, BQ2562X_CHRG_CTRL_1,
					     BQ2562X_CHRG_HIZ, 0);
	if (ret) {
		goto cleanup;
	}

	/* Disable OTG mode */
	ret = mfd_bq2562x_reg_update_byte_dt(config->mfd, BQ2562X_CHRG_CTRL_3,
					     BQ2562X_CTRL3_EN_OTG, 0);
	if (ret) {
		goto cleanup;
	}

	/* Enable charging mode */
	ret = mfd_bq2562x_reg_update_byte_dt(config->mfd, BQ2562X_CHRG_CTRL_1,
					     BQ2562X_CHRG_EN, BQ2562X_CHRG_EN);
	if (ret) {
		goto cleanup;
	}

	/* To determine the voltage and current value of the pd partner */
	ret = bq25620_get_ibus_adc(dev, &ibus_ua);
	if (ret) {
		LOG_ERR("Failed to read the IBUS ADC ret : %d", ret);
		return ret;
	}

	ret = bq25620_get_vbus_adc(dev, &vbus_uv);
	if (ret) {
		LOG_ERR("Failed to read the VBUS ADC ret : %d", ret);
		return ret;
	}

	partner_state.bc12_role = BC12_PORTABLE_DEVICE;
	ret = bq25620_get_pd_type(dev, &partner_state);
	if (ret) {
		return ret;
	}
	partner_state.current_ua = ibus_ua;
	partner_state.voltage_uv = vbus_uv;

	bq25620_update_charging_partner(dev, &partner_state);

cleanup:
	mfd_bq2562x_enable_interrupt_pin(config->mfd, true);
	return ret;
}

static int bq25620_set_mode(const struct device *dev, enum bq25620_mode mode)
{
	int ret;

	ret = bq25620_set_votg(dev, BQ2562X_VOLT_UV);
	if (ret) {
		return ret;
	}
	switch(mode){
	case BQ2562X_POWER_DOWN:
		ret = bq25620_set_iotg(dev, BQ2562X_POWER_NONE);
		if (ret) {
			return ret;
		}
		ret = bq25620_set_votg(dev, BQ2562X_POWER_NONE);
		if (ret) {
			return ret;
		}
		break;
	case BQ2562X_SDP_HOST_MODE:
		ret = bq25620_set_iotg(dev, BQ2562X_SDP_CURR_UA);
		if (ret) {
			return ret;
		}
		break;
	case BQ2562X_DCP_HOST_MODE:
		__fallthrough;
	case BQ2562X_CDP_HOST_MODE:
		ret = bq25620_set_iotg(dev, BQ2562X_CDP_CURR_UA);
		if (ret) {
			return ret;
		}
		break;
	default:
		LOG_ERR("Not supporting mode !!");
		return -EINVAL;
	};
	return ret;
}

static int bq25620_set_charging_mode(const struct device *dev)
{
	const struct bc12_bq25620_config *config = dev->config;
	struct bc12_bq25620_data *data = dev->data;
	struct bc12_partner_state partner_state;
	int32_t ibus_ua;
	int ret;

	if (data->partner_state.bc12_role == BC12_CHARGING_PORT) {
		/* Device is already in the same mode */
		return 0;
	}

	ret = bq25620_set_mode(dev, charging_mode_to_host_mode[config->charging_mode]);
	if (ret) {
		return ret;
	}

	/* Disable interrupts during mode change */
	mfd_bq2562x_enable_interrupt_pin(config->mfd, false);

	/* Enable auto indet to start dpdm detection */
	ret = mfd_bq2562x_reg_update_byte_dt(config->mfd, BQ2562X_TIMER_CTRL,
					     BQ2562X_TIMER_AUTO_INDET, BQ2562X_TIMER_AUTO_INDET);
	if (ret) {
		goto cleanup;
	}

	/* Disable HIZ mode */
	ret = mfd_bq2562x_reg_update_byte_dt(config->mfd, BQ2562X_CHRG_CTRL_1, BQ2562X_CHRG_HIZ, 0);
	if (ret) {
		goto cleanup;
	}

	/* Disable charging mode */
	ret = mfd_bq2562x_reg_update_byte_dt(config->mfd, BQ2562X_CHRG_CTRL_1, BQ2562X_CHRG_EN, 0);
	if (ret) {
		goto cleanup;
	}

	/* Enable OTG mode */
	ret = mfd_bq2562x_reg_update_byte_dt(config->mfd, BQ2562X_CHRG_CTRL_3,
					     BQ2562X_CTRL3_EN_OTG, BQ2562X_CTRL3_EN_OTG);
	if (ret) {
		goto cleanup;
	}

	/* To determine pd is connected or not */
	ret = bq25620_get_ibus_adc(dev, &ibus_ua);
	if (ret) {
		LOG_ERR(" Failed to read the IBUS ADC ret : %d", ret);
		return ret;
	}

	if (ibus_ua < 0) {
		partner_state.pd_partner_connected = true;
	} else {
		partner_state.pd_partner_connected = false;
	}

	partner_state.bc12_role = BC12_CHARGING_PORT;
	bq25620_update_charging_partner(dev, &partner_state);

cleanup:
	mfd_bq2562x_enable_interrupt_pin(config->mfd, true);
	return ret;
}

static int bq25620_set_role(const struct device *dev, const enum bc12_role role)
{
	switch (role) {
	case BC12_DISCONNECTED:
		return bq25620_disconnect(dev);
	case BC12_PORTABLE_DEVICE:
		return bq25620_set_portable_device(dev);
	case BC12_CHARGING_PORT:
		return bq25620_set_charging_mode(dev);
	default:
		LOG_ERR("Unsupported BC12 role : %d", role);
		return -EINVAL;
	};

	return 0;
}

static int bq25620_set_result_cb(const struct device *dev, bc12_callback_t cb,
				 void *const user_data)
{
	struct bc12_bq25620_data *data = dev->data;

	data->result_cb = cb;
	data->result_cb_data = user_data;

	return 0;
}

static int bc12_bq25620_init(const struct device *dev)
{
	const struct bc12_bq25620_config *config = dev->config;
	struct bc12_bq25620_data *data = dev->data;

	/* interrupt handler register for bc12 */
	data->bq25620_cb.cb = bq25620_int_work_handler;
	data->bq25620_cb.dev = dev;
	mfd_bq2562x_register_interrupt_callback(config->mfd, &data->bq25620_cb);

	return 0;
}

static DEVICE_API(bc12, bc12_bq25620_driver_api) = {
	.set_role = bq25620_set_role,
	.set_result_cb = bq25620_set_result_cb,
};

#define BQ25620_INIT(n)										\
												\
	static const struct bc12_bq25620_config bq25620_config_##n = {				\
		.mfd = DEVICE_DT_GET(DT_INST_PARENT(n)),					\
		.charging_mode = DT_INST_STRING_UPPER_TOKEN(n, charging_mode),			\
	};											\
												\
	static struct bc12_bq25620_data bq25620_data_##n;					\
												\
	DEVICE_DT_INST_DEFINE(n, bc12_bq25620_init, NULL, &bq25620_data_##n,			\
			      &bq25620_config_##n, POST_KERNEL,					\
			      CONFIG_BC12_BQ25620_INIT_PRIORITY, &bc12_bq25620_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BQ25620_INIT)
