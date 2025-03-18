/*
 * Copyright 2025 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_bq2562x

#include "charger_bq2562x.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ti_bq24190);

struct bq2562x_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec ce_gpio;
	struct gpio_dt_spec int_gpio;
};

#define I2C_NAME_SIZE	20
struct bq2562x_data {
	const struct device *dev;
	struct gpio_callback gpio_cb;
	charger_status_notifier_t charger_status_notifier;
	charger_online_notifier_t charger_online_notifier;
	struct k_work int_routine_work;
	char model_name[I2C_NAME_SIZE];
	uint32_t constant_charge_current_max_ua;
	uint32_t constant_charge_voltage_max_uv;
	uint32_t precharge_current_ua;
	uint32_t charge_term_current_ua;

	/* TI/Chip specific */
	uint32_t min_sys_voltage_uv;
	uint32_t input_voltage_min_uv;
	uint32_t input_current_max_ua;
	uint32_t thermal_regulation_threshold;
	uint32_t switching_converter_freq;
	uint32_t switching_converter_strength;
	enum charger_status state;
	enum charger_online online;
};

enum bq2562x_id {
	BQ25620,
	BQ25622,
};

static bool bq2562x_get_charge_enable(const struct device *dev)
{
	int ret;
	int ce_pin = 1;
	int charger_enable;
	uint8_t chrg_ctrl_0;
	const struct bq2562x_config *const config = dev->config;

	if (config->ce_gpio.port != NULL) {
		ce_pin = !gpio_pin_get_dt(&config->ce_gpio);
	}

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_CHRG_CTRL_1, &chrg_ctrl_0);
	if (ret) {
		return ret;
	}

	charger_enable = chrg_ctrl_0 & BQ2562X_CHRG_EN;
	if (charger_enable) {
		if (config->ce_gpio.port && !ce_pin) {
			return true;
		}
	}

	return false;
}

static int bq2562x_set_charge_enable(const struct device *dev, const bool enable)
{
	int ret;
	const struct bq2562x_config *const config = dev->config;

	if (config->ce_gpio.port != NULL) {
		ret = gpio_pin_set_dt(&config->ce_gpio, enable);
		if (ret) {
			return ret;
		}
	}

	return i2c_reg_update_byte_dt(&config->i2c, BQ2562X_CHRG_CTRL_1, BQ2562X_CHRG_EN,
					enable ? BQ2562X_CHRG_EN : 0);
}

/* Charge Current Limit */
static int bq2562x_get_ichrg_curr(const struct device *dev, uint32_t *current_ua)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	int ichg;
	uint8_t ichg_lsb, ichg_msb;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_CHRG_I_LIM_MSB, &ichg_msb);
	if (ret)
		return ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_CHRG_I_LIM_LSB, &ichg_lsb);
	if (ret)
		return ret;

	ichg = ((ichg_msb << 8) | ichg_lsb) >> BQ2562X_ICHG_I_MOVE_STEP;
	*current_ua = ichg * BQ2562X_ICHG_I_STEP_uA;

	return 0;
}

static int bq2562x_set_ichrg_curr(const struct device *dev, int chrg_curr)
{
	const struct bq2562x_config *const config = dev->config;
	struct bq2562x_data *data = dev->data;
	int chrg_curr_max = data->constant_charge_current_max_ua;
	int ichg;
	uint8_t ichg_msb, ichg_lsb;
	int ret;

	chrg_curr = CLAMP(chrg_curr, BQ2562X_ICHG_I_MIN_uA,
					chrg_curr_max);

	bq2562x_set_charge_enable(dev, 0);

	ichg = ((chrg_curr / BQ2562X_ICHG_I_STEP_uA) << BQ2562X_ICHG_I_MOVE_STEP);
	ichg_msb = (ichg >> 8) & 0xff;
	ret = i2c_reg_write_byte_dt(&config->i2c, BQ2562X_CHRG_I_LIM_MSB, ichg_msb);
	if (ret) {
		return ret;
	}

	ichg_lsb = ichg & 0xff;

	ret = i2c_reg_write_byte_dt(&config->i2c, BQ2562X_CHRG_I_LIM_LSB, ichg_lsb);
	if (ret) {
		return ret;
	}

	return bq2562x_set_charge_enable(dev, 1);
}

/* Charge Voltage Limit */
static int bq2562x_get_chrg_volt(const struct device *dev, uint32_t *voltage_uv)
{
	int ret;
	const struct bq2562x_config *const config = dev->config;
	uint8_t chrg_volt_lsb, chrg_volt_msb;
	int chrg_volt;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_CHRG_V_LIM_LSB, &chrg_volt_lsb);
	if (ret) {
		return ret;
	}

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_CHRG_V_LIM_MSB, &chrg_volt_msb);
	if (ret) {
		return ret;
	}

	chrg_volt = ((chrg_volt_msb << 8) | chrg_volt_lsb) >> BQ2562X_VREG_V_MOVE_STEP;
	*voltage_uv = chrg_volt * BQ2562X_VREG_V_STEP_uV;

	return 0;
}

static int bq2562x_set_chrg_volt(const struct device *dev, int chrg_volt)
{
	const struct bq2562x_config *const config = dev->config;
	struct bq2562x_data *data = dev->data;
	int chrg_volt_max = data->constant_charge_voltage_max_uv;
	uint8_t chrg_volt_lsb, chrg_volt_msb;
	int vlim;
	int ret;

	chrg_volt = CLAMP(chrg_volt, BQ2562X_VREG_V_MIN_uV, chrg_volt_max);

	vlim = (chrg_volt / BQ2562X_VREG_V_STEP_uV) << BQ2562X_VREG_V_MOVE_STEP;
	chrg_volt_msb = (vlim >> 8) & 0xff;
	ret = i2c_reg_write_byte_dt(&config->i2c, BQ2562X_CHRG_V_LIM_MSB, chrg_volt_msb);
	if (ret) {
		return ret;
	}

	chrg_volt_lsb = vlim & 0xff;

	return i2c_reg_write_byte_dt(&config->i2c, BQ2562X_CHRG_V_LIM_LSB, chrg_volt_lsb);
}

/* Input Current Limit */
static int bq2562x_get_input_curr_lim(const struct device *dev, uint32_t *current_ua)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	uint8_t ilim_msb, ilim_lsb;
	uint16_t ilim;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_INPUT_I_LIM_LSB, &ilim_lsb);
	if (ret)
		return ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_INPUT_I_LIM_MSB, &ilim_msb);
	if (ret)
		return ret;

	ilim = ((ilim_msb << 8) | ilim_lsb) >> BQ2562X_IINDPM_I_MOVE_STEP;
	*current_ua = ilim * BQ2562X_IINDPM_I_STEP_uA;

	return 0;
}

static int bq2562x_set_input_curr_lim(const struct device *dev, int iindpm)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	int ilim;
	uint8_t ilim_lsb, ilim_msb;

	iindpm = CLAMP(iindpm, BQ2562X_IINDPM_I_MIN_uA,
						BQ2562X_IINDPM_I_MAX_uA);

	ilim = (iindpm / BQ2562X_IINDPM_I_STEP_uA) << BQ2562X_IINDPM_I_MOVE_STEP;

	ilim_lsb = ilim & 0xff;
	ret = i2c_reg_write_byte_dt(&config->i2c, BQ2562X_INPUT_I_LIM_LSB, ilim_lsb);
	if (ret)
		return ret;

	ilim_msb = (ilim >> 8) & 0xff;
	ret = i2c_reg_write_byte_dt(&config->i2c, BQ2562X_INPUT_I_LIM_MSB, ilim_msb);

	return ret;
}

/* Input Voltage Limit */
static int bq2562x_get_input_volt_lim(const struct device *dev, uint32_t *voltage_uv)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	int vlim;
	uint8_t vlim_lsb, vlim_msb;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_INPUT_V_LIM_LSB, &vlim_lsb);
	if (ret)
		return ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_INPUT_V_LIM_MSB, &vlim_msb);
	if (ret)
		return ret;

	vlim = ((vlim_msb << 8) | vlim_lsb) >> BQ2562X_VINDPM_V_MOVE_STEP;
	*voltage_uv = vlim * BQ2562X_VINDPM_V_STEP_uV;

	return 0;
}

static int bq2562x_set_input_volt_lim(const struct device *dev, int vindpm)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	uint8_t vlim_lsb, vlim_msb;
	int vlim;

	vindpm = CLAMP(vindpm, BQ2562X_VINDPM_V_MIN_uV,
						BQ2562X_VINDPM_V_MAX_uV);

	vlim = (vindpm / BQ2562X_VINDPM_V_STEP_uV) << BQ2562X_VINDPM_V_MOVE_STEP;

	vlim_msb = (vlim >> 8) & 0xff;

	ret = i2c_reg_write_byte_dt(&config->i2c, BQ2562X_INPUT_V_LIM_MSB, vlim_msb);
	if (ret)
		return ret;

	vlim_lsb = vlim & 0xff;

	return i2c_reg_write_byte_dt(&config->i2c, BQ2562X_INPUT_V_LIM_LSB, vlim_lsb);
}

/* Minimal System Voltage */
static int bq2562x_set_min_sys_volt(const struct device *dev, int vsysmin)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	uint8_t vlim_lsb, vlim_msb;
	int vlim;

	vsysmin = CLAMP(vsysmin, BQ2562X_VSYSMIN_V_MIN_uV,
						BQ2562X_VSYSMIN_V_MAX_uV);

	vlim = (vsysmin / BQ2562X_VSYSMIN_V_MOVE_STEP_uV) << BQ2562X_VSYSMIN_V_MOVE_STEP;

	vlim_msb = (vlim >> 8) & 0xff;

	ret = i2c_reg_write_byte_dt(&config->i2c, BQ2562X_MIN_SYS_V_MSB, vlim_msb);
	if (ret)
		return ret;

	vlim_lsb = vlim & 0xff;

	return i2c_reg_write_byte_dt(&config->i2c, BQ2562X_MIN_SYS_V_LSB, vlim_lsb);
}

/* Pre-charge Control */
static int bq2562x_get_prechrg_curr(const struct device *dev, uint32_t *current_ua)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	uint8_t prechrg_curr_lsb, prechrg_curr_msb;
	uint16_t prechrg_curr;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_PRECHRG_CTRL_LSB, &prechrg_curr_lsb);
	if (ret)
		return ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_PRECHRG_CTRL_MSB, &prechrg_curr_msb);
	if (ret)
		return ret;

	prechrg_curr = ((prechrg_curr_msb << 8) | prechrg_curr_lsb) >> BQ2562X_PRECHRG_I_MOVE_STEP;
	*current_ua = prechrg_curr * BQ2562X_PRECHRG_I_STEP_uA;

	return 0;
}

static int bq2562x_set_prechrg_curr(const struct device *dev, int pre_current)
{
	const struct bq2562x_config *const config = dev->config;
	int reg_val;
	uint8_t prechrg_curr_msb, prechrg_curr_lsb;
	int ret;

	pre_current = CLAMP(pre_current, BQ2562X_PRECHRG_I_MIN_uA,
					BQ2562X_PRECHRG_I_MAX_uA);

	bq2562x_set_charge_enable(dev, 0);

	reg_val = (pre_current / BQ2562X_PRECHRG_I_STEP_uA) << BQ2562X_PRECHRG_I_MOVE_STEP;

	prechrg_curr_msb = (reg_val >> 8) & 0xff;
	ret = i2c_reg_write_byte_dt(&config->i2c, BQ2562X_PRECHRG_CTRL_MSB, prechrg_curr_msb);
	if (ret)
		return ret;

	prechrg_curr_lsb = reg_val & 0xff;

	ret = i2c_reg_write_byte_dt(&config->i2c, BQ2562X_PRECHRG_CTRL_LSB, prechrg_curr_lsb);
	if (ret)
		return ret;

	return bq2562x_set_charge_enable(dev, 1);
}

/* Termination Control */
static int bq2562x_get_term_curr(const struct device *dev, uint32_t *current_ua)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	uint8_t iterm_lsb, iterm_msb;
	uint16_t iterm;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_TERM_CTRL_LSB, &iterm_lsb);
	if (ret)
		return ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_TERM_CTRL_MSB, &iterm_msb);
	if (ret)
		return ret;

	iterm = ((iterm_msb << 8) | iterm_lsb) >> BQ2562X_TERMCHRG_I_MOVE_STEP;
	*current_ua = iterm * BQ2562X_TERMCHRG_I_STEP_uA;

	return 0;
}

static int bq2562x_set_term_curr(const struct device *dev, int term_current)
{
	const struct bq2562x_config *const config = dev->config;
	int reg_val;
	uint8_t term_curr_msb, term_curr_lsb;
	int ret;

	term_current = CLAMP(term_current, BQ2562X_TERMCHRG_I_MIN_uA,
					BQ2562X_TERMCHRG_I_MAX_uA);

	reg_val = (term_current / BQ2562X_TERMCHRG_I_STEP_uA) << BQ2562X_TERMCHRG_I_MOVE_STEP;

	term_curr_msb = (reg_val >> 8) & 0xff;
	ret = i2c_reg_write_byte_dt(&config->i2c, BQ2562X_TERM_CTRL_MSB, term_curr_msb);
	if (ret)
		return ret;

	term_curr_lsb = reg_val & 0xff;

	return i2c_reg_write_byte_dt(&config->i2c, BQ2562X_TERM_CTRL_LSB, term_curr_lsb);
}

static int bq2562x_get_vbat_adc(const struct device *dev, uint32_t *vbat)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	uint8_t vbat_adc_lsb, vbat_adc_msb;
	uint16_t vbat_adc;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_ADC_VBAT_LSB, &vbat_adc_lsb);
	if (ret)
		return ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_ADC_VBAT_MSB, &vbat_adc_msb);
	if (ret)
		return ret;

	vbat_adc = ((vbat_adc_msb << 8) | vbat_adc_lsb) >> BQ2562X_ADC_VBAT_MOVE_STEP;
	*vbat = vbat_adc * BQ2562X_ADC_VBAT_STEP_uV;

	return 0;
}

static int bq2562x_get_vbus_adc(const struct device *dev, uint32_t *volt)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	uint8_t vbus_adc_lsb, vbus_adc_msb;
	uint16_t vbus_adc;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_ADC_VBUS_LSB, &vbus_adc_lsb);
	if (ret)
		return ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_ADC_VBUS_MSB, &vbus_adc_msb);
	if (ret)
		return ret;

	vbus_adc = ((vbus_adc_msb << 8) | vbus_adc_lsb) >> BQ2562X_ADC_VBUS_MOVE_STEP;
	*volt = vbus_adc * BQ2562X_ADC_VBUS_STEP_uV;

	return 0;
}

static int bq2562x_get_ibat_adc(const struct device *dev, uint32_t *ibat)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	uint8_t ibat_adc_lsb, ibat_adc_msb;
	uint16_t ibat_adc;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_ADC_IBAT_LSB, &ibat_adc_lsb);
	if (ret)
		return ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_ADC_IBAT_MSB, &ibat_adc_msb);
	if (ret)
		return ret;

	ibat_adc = ((ibat_adc_msb << 8) | ibat_adc_lsb) >> BQ2562X_ADC_IBAT_MOVE_STEP;
	*ibat = ibat_adc * BQ2562X_ADC_IBAT_STEP_uV;

	return 0;
}

static int bq2562x_get_ibus_adc(const struct device *dev, uint32_t *ibus)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	uint8_t ibus_adc_lsb, ibus_adc_msb;
	int ibus_adc;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_ADC_IBUS_LSB, &ibus_adc_lsb);
	if (ret)
		return ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_ADC_IBUS_MSB, &ibus_adc_msb);
	if (ret)
		return ret;

	ibus_adc = ((ibus_adc_msb << 8) | ibus_adc_lsb) >> BQ2562X_ADC_IBUS_MOVE_STEP;
	*ibus = ibus_adc * BQ2562X_ADC_CURR_STEP_uA;

	return 0;
}

static int bq2562x_get_online_status(const struct device *dev, enum charger_online *online)
{
	const struct bq2562x_config *const config = dev->config;
	int ret;
	uint8_t chrg_stat_1;
	int online_status;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_CHRG_STAT_1, &chrg_stat_1);
	if (ret)
		return ret;

	online_status = chrg_stat_1 & BQ2562X_VBUS_STAT_MSK;
	if (!online_status || (online_status == BQ2562X_OTG_MODE))
		*online = CHARGER_ONLINE_OFFLINE;
	else
		*online = CHARGER_ONLINE_FIXED;

	return 0;
}

static int bq2562x_get_health(const struct device *dev, enum charger_health *health)
{
	int ret;
	uint8_t fault;
	const struct bq2562x_config *const config = dev->config;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_FAULT_STAT_0, &fault);
	if (ret)
		return ret;

	if (fault & BQ2562X_TEMP_MASK) {
		switch (fault & BQ2562X_TEMP_MASK) {
		case BQ2562X_TEMP_TS_NORMAL:
			*health = CHARGER_HEALTH_GOOD;
			break;
		case BQ2562X_TEMP_COLD:
			*health = CHARGER_HEALTH_COLD;
			break;
		case BQ2562X_TEMP_HOT:
			*health = CHARGER_HEALTH_HOT;
			break;
		case BQ2562X_TEMP_COOL:
			__fallthrough;
		case BQ2562X_TEMP_PRECOOL:
			*health = CHARGER_HEALTH_COOL;
			break;
		case BQ2562X_TEMP_WARM:
			__fallthrough;
		case BQ2562X_TEMP_PREWARM:
			*health = CHARGER_HEALTH_WARM;
			break;
		case BQ2562X_TEMP_PIN_BIAS_REF_FAULT:
			*health = CHARGER_HEALTH_DEAD;
			break;
		}
	} else if (fault & BQ2562X_TSHUT_STAT) {
		*health = CHARGER_HEALTH_OVERHEAT;
	} else if (fault & (BQ2562X_OTG_FAULT_STAT | BQ2562X_SYS_FAULT_STAT | BQ2562X_BAT_FAULT_STAT | BQ2562X_VBUS_FAULT_STAT)) {
		*health = CHARGER_HEALTH_OVERVOLTAGE;
	} else {
		*health = CHARGER_HEALTH_UNKNOWN;
	}

	return 0;
}

static int bq2562x_get_charger_type(const struct device *dev, enum charger_charge_type *type)
{
	int ret;
	uint8_t chrg_stat_1;
	const struct bq2562x_config *const config = dev->config;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_CHRG_STAT_1, &chrg_stat_1);
	if (ret)
		return ret;

	if (bq2562x_get_charge_enable(dev)) {
		chrg_stat_1 = FIELD_GET(BQ2562X_CHG_STAT_MSK, chrg_stat_1);
		switch (chrg_stat_1) {
		case BQ2562X_NOT_CHRGING:
			*type = CHARGER_CHARGE_TYPE_NONE;
			break;
		case BQ2562X_TAPER_CHRG:
			*type = CHARGER_CHARGE_TYPE_STANDARD;
			break;
		case BQ2562X_TOP_OFF_CHRG:
			__fallthrough;
		case BQ2562X_TRICKLE_CHRG:
			/* FIXME: trickle, pre-charge, fast all in same bit, no way to differentiate */
			*type = CHARGER_CHARGE_TYPE_TRICKLE;
			break;
		}
	}
	else
		*type = CHARGER_CHARGE_TYPE_UNKNOWN;

	return 0;
}

static int bq2562x_get_charger_status(const struct device *dev, enum charger_status *charge_status)
{
	int ret;
	uint8_t chrg_stat_1;
	uint8_t type, status;
	int chrg_volt;
	int vbat_adc;
	const struct bq2562x_config *const config = dev->config;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_CHRG_STAT_1, &chrg_stat_1);
	if (ret)
		return ret;

	ret = bq2562x_get_vbat_adc(dev, &vbat_adc);
	if (ret)
		return ret;

	if (bq2562x_get_charge_enable(dev))
		status = FIELD_GET(BQ2562X_CHG_STAT_MSK, chrg_stat_1);
	else
		status = BQ2562X_NOT_CHRGING;

	type = FIELD_GET(BQ2562X_VBUS_STAT_MSK, chrg_stat_1);

	if (!type || (type == BQ2562X_OTG_MODE))
		*charge_status = CHARGER_STATUS_DISCHARGING;
	else if (!status) {
		*charge_status = CHARGER_STATUS_NOT_CHARGING;
		ret = bq2562x_get_chrg_volt(dev, &chrg_volt);
		if (ret < 0)
			return 0;
		if (chrg_volt >= (vbat_adc - BQ2562X_ADC_VBAT_STEP_uV)) {
			*charge_status = CHARGER_STATUS_FULL;
		}
	}
	else
		*charge_status = CHARGER_STATUS_CHARGING;

	return 0;
}

static int bq2562x_get_usb_type(const struct device *dev, enum charger_usb_type *type)
{
	int ret;
	uint8_t chrg_stat_1;
	const struct bq2562x_config *const config = dev->config;

	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_CHRG_STAT_1, &chrg_stat_1);
	if (ret) {
		return ret;
	}

	chrg_stat_1 = FIELD_GET(BQ2562X_VBUS_STAT_MSK, chrg_stat_1);
	switch (chrg_stat_1) {
	case BQ2562X_USB_SDP:
		*type = CHARGER_USB_TYPE_SDP;
		break;
	case BQ2562X_USB_CDP:
		*type = CHARGER_USB_TYPE_CDP;
		break;
	case BQ2562X_USB_DCP:
		*type = CHARGER_USB_TYPE_DCP;
		break;
	case BQ2562X_OTG_MODE:
		*type = CHARGER_USB_TYPE_ACA;
		break;
	case BQ2562X_UNKNOWN_500MA:
		__fallthrough;
	case BQ2562X_NON_STANDARD:
		__fallthrough;
	case BQ2562X_HVDCP: /* TODO */
		__fallthrough;
	default:
		*type = CHARGER_USB_TYPE_UNKNOWN;
	}

	return 0;
}

static int bq2562x_get_prop(const struct device *dev, charger_prop_t prop,
			    union charger_propval *val)
{
	struct bq2562x_data *data = dev->data;

	switch (prop) {
	case CHARGER_PROP_ONLINE:
		return bq2562x_get_online_status(dev, &val->online);
	case CHARGER_PROP_MANUFACTURER:
		val->manufacturer = BQ2562X_MANUFACTURER;
		break;
	case CHARGER_PROP_MODEL_NAME:
		val->model = data->model_name;
		break;
	case CHARGER_PROP_CHARGE_TYPE:
		return bq2562x_get_charger_type(dev, &val->charge_type);
	case CHARGER_PROP_HEALTH:
		return bq2562x_get_health(dev, &val->health);
	case CHARGER_PROP_STATUS:
		return bq2562x_get_charger_status(dev, &val->status);
	case CHARGER_PROP_USB_TYPE:
		return bq2562x_get_usb_type(dev, &val->usb_type);
	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA:
		return bq2562x_get_ichrg_curr(dev, &val->const_charge_current_ua);
	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
		return bq2562x_get_chrg_volt(dev, &val->const_charge_voltage_uv);
	case CHARGER_PROP_PRECHARGE_CURRENT_UA:
		return bq2562x_get_prechrg_curr(dev, &val->precharge_current_ua);
	case CHARGER_PROP_CHARGE_TERM_CURRENT_UA:
		return bq2562x_get_term_curr(dev, &val->charge_term_current_ua);
	case CHARGER_PROP_BATTERY_VOLTAGE_NOW:
		return bq2562x_get_vbat_adc(dev, &val->bat_voltage_now_uv);
	case CHARGER_PROP_BATTERY_CURRENT_NOW:
		return bq2562x_get_ibat_adc(dev, &val->bat_current_now_ua);
	case CHARGER_PROP_VOLTAGE_NOW:
		return bq2562x_get_vbus_adc(dev, &val->voltage_now_uv);
	case CHARGER_PROP_CURRENT_NOW:
		return bq2562x_get_ibus_adc(dev, &val->current_now_ua);
	case CHARGER_PROP_INPUT_REGULATION_CURRENT_UA:
		return bq2562x_get_input_curr_lim(dev, &val->input_current_regulation_current_ua);
	case CHARGER_PROP_INPUT_REGULATION_VOLTAGE_UV:
		return bq2562x_get_input_volt_lim(dev, &val->input_voltage_regulation_voltage_uv);
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int bq2562x_set_prop(const struct device *dev, charger_prop_t prop,
			    const union charger_propval *val)
{
	struct bq2562x_data *data = dev->data;

	switch (prop) {
	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA:
		return bq2562x_set_ichrg_curr(dev, val->const_charge_current_ua);
	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
		return bq2562x_set_chrg_volt(dev, val->const_charge_voltage_uv);
	case CHARGER_PROP_PRECHARGE_CURRENT_UA:
		return bq2562x_set_prechrg_curr(dev, val->precharge_current_ua);
	case CHARGER_PROP_CHARGE_TERM_CURRENT_UA:
		return bq2562x_set_term_curr(dev, val->charge_term_current_ua);
	case CHARGER_PROP_INPUT_REGULATION_CURRENT_UA:
		return bq2562x_set_input_curr_lim(dev, val->input_current_regulation_current_ua);
	case CHARGER_PROP_INPUT_REGULATION_VOLTAGE_UV:
		return bq2562x_set_input_volt_lim(dev, val->input_voltage_regulation_voltage_uv);
	case CHARGER_PROP_STATUS_NOTIFICATION:
		data->charger_status_notifier = val->status_notification;
		break;
	case CHARGER_PROP_ONLINE_NOTIFICATION:
		data->charger_online_notifier = val->online_notification;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2562x_validate_dt(struct bq2562x_data *data)
{
	if (!IN_RANGE(data->min_sys_voltage_uv,
			BQ2562X_VSYSMIN_V_MIN_uV,
			BQ2562X_VSYSMIN_V_MAX_uV)) {
		data->min_sys_voltage_uv = BQ2562X_VSYSMIN_V_DEF_uV;
	}

	if (!IN_RANGE(data->input_voltage_min_uv,
			BQ2562X_VINDPM_V_MIN_uV,
			BQ2562X_VINDPM_V_MAX_uV)) {
		data->input_voltage_min_uv = BQ2562X_VINDPM_V_DEF_uV;
	}

	if (!IN_RANGE(data->input_current_max_ua,
			BQ2562X_IINDPM_I_MIN_uA,
			BQ2562X_IINDPM_I_MAX_uA)) {
		data->input_current_max_ua = BQ2562X_IINDPM_I_DEF_uA;
	}

	return 0;
}

static int bq2562x_set_heat_mgmt(const struct device *dev)
{
	const struct bq2562x_config *const config = dev->config;
	struct bq2562x_data *data = dev->data;
	int ret;

	ret = i2c_reg_update_byte_dt(&config->i2c, BQ2562X_CHRG_CTRL_2,
					BQ2562X_CTRL2_SET_CONV_STRN,
					data->switching_converter_strength << 2);
	if (ret) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, BQ2562X_CHRG_CTRL_2,
					BQ2562X_CTRL2_SET_CONV_FREQ,
					data->switching_converter_freq << 4);
	if (ret) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, BQ2562X_CHRG_CTRL_2,
					BQ2562X_CTRL2_TREG,
					data->thermal_regulation_threshold << 6);
	if (ret) {
		return ret;
	}

	return bq2562x_set_min_sys_volt(dev, data->min_sys_voltage_uv);
}

static int bq2562x_hw_init(const struct device *dev)
{
	int ret;
	struct bq2562x_data *data = dev->data;
	const struct bq2562x_config *const config = dev->config;

	ret = i2c_reg_update_byte_dt(&config->i2c, BQ2562X_CHRG_CTRL_2, BQ2562X_CTRL2_REG_RST,
					BQ2562X_CTRL2_REG_RST);
	if (ret) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, BQ2562X_NTC_CTRL_0, BQ2562X_NTC_MASK,
					BQ2562X_NTC_MASK);
	if (ret) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, BQ2562X_CHRG_CTRL_1, BQ2562X_WATCHDOG_MASK,
					BQ2562X_WATCHDOG_DIS);
	if (ret) {
		return ret;
	}

	uint8_t tmp;
	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_CHRG_CTRL_2, &tmp);
	printk("ctrl 2: %x\n", tmp);
	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_NTC_CTRL_0, &tmp);
	printk("ntc 0: %x\n", tmp);
	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_CHRG_CTRL_1, &tmp);
	printk("ctrl 1: %x\n", tmp);

	ret = bq2562x_set_ichrg_curr(dev, data->constant_charge_current_max_ua);
	if (ret) {
		return ret;
	}

	ret = bq2562x_set_chrg_volt(dev, data->constant_charge_voltage_max_uv);
	if (ret) {
		return ret;
	}

	ret = bq2562x_set_prechrg_curr(dev, data->precharge_current_ua);
	if (ret) {
		return ret;
	}

	ret = bq2562x_set_term_curr(dev, data->charge_term_current_ua);
	if (ret) {
		return ret;
	}

	ret = bq2562x_set_input_volt_lim(dev, data->input_voltage_min_uv);
	if (ret) {
		return ret;
	}

	ret = bq2562x_set_input_curr_lim(dev, data->input_current_max_ua);
	if (ret) {
		return ret;
	}

	ret = bq2562x_set_heat_mgmt(dev);
	if (ret) {
		return ret;
	}

	return 0;
}

static int bq2562x_enable_interrupt_pin(const struct device *dev, bool enabled)
{
	const struct bq2562x_config *const config = dev->config;
	gpio_flags_t flags;
	int ret;

	flags = enabled ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE;

	ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, flags);
	if (ret < 0) {
		LOG_ERR("Could not %s interrupt GPIO callback: %d", enabled ? "enable" : "disable",
			ret);
	}

	return ret;
}

static void bq2562x_int_routine_work_handler(struct k_work *work)
{
	struct bq2562x_data *data = CONTAINER_OF(work, struct bq2562x_data, int_routine_work);
	union charger_propval val;
	int ret;

	if (data->charger_status_notifier != NULL) {
		ret = bq2562x_get_charger_status(data->dev, &val.status);
		if (!ret) {
			data->charger_status_notifier(val.status);
		}
	}

	if (data->charger_online_notifier != NULL) {
		ret = bq2562x_get_online_status(data->dev, &val.online);
		if (!ret) {
			data->charger_online_notifier(val.online);
		}
	}
	(void) bq2562x_enable_interrupt_pin(data->dev, true);
}

static void bq2562x_gpio_callback(const struct device *dev, struct gpio_callback *cb,
				   uint32_t pins)
{
	struct bq2562x_data *data = CONTAINER_OF(cb, struct bq2562x_data, gpio_cb);
	int ret;

	(void) bq2562x_enable_interrupt_pin(data->dev, false);

	ret = k_work_submit(&data->int_routine_work);
	if (ret < 0) {
		LOG_WRN("Could not submit int work: %d", ret);
	}
}

static int bq2562x_configure_interrupt(const struct device *dev)
{
	const struct bq2562x_config *const config = dev->config;
	struct bq2562x_data *data = dev->data;
	int ret;

	k_work_init(&data->int_routine_work, bq2562x_int_routine_work_handler);
	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("Interrupt GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure interrupt GPIO");
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, bq2562x_gpio_callback, BIT(config->int_gpio.pin));
	ret = gpio_add_callback_dt(&config->int_gpio, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Could not add interrupt GPIO callback");
		return ret;
	}

	return 0;
}

static int bq2562x_init(const struct device *dev)
{
	const struct bq2562x_config *const config = dev->config;
	struct bq2562x_data *data = dev->data;
	uint8_t val;
	int ret;

	data->dev = dev;
	ret = i2c_reg_read_byte_dt(&config->i2c, BQ2562X_PART_INFO, &val);
	if (ret) {
		return ret;
	}

	val = FIELD_GET(BQ2562X_PART_NO_MASK, val);
	switch (val) {
	case BQ25620:
		strncpy(data->model_name, "BQ25620", I2C_NAME_SIZE);
		break;
	case BQ25622:
		return -ENOTSUP;
	default:
		LOG_ERR("Error unknown model: 0x%02x\n", val);
		return -ENODEV;
	}

	/* charge enable */
	if (config->ce_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->ce_gpio)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->ce_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}
	} else {
		LOG_DBG("Assuming charge enable pin is pulled low");
	}

	if (config->int_gpio.port != NULL) {
		ret = bq2562x_configure_interrupt(dev);
		if (ret) {
			return ret;
		}
		printk("INT configured\n");
	}

	/* DT sanity */
	ret = bq2562x_validate_dt(data);
	if (ret) {
		return ret;
	}

	return bq2562x_hw_init(dev);
}

static DEVICE_API(charger, bq2562x_driver_api) = {
	.get_property = bq2562x_get_prop,
	.set_property = bq2562x_set_prop,
	.charge_enable = bq2562x_set_charge_enable,
};

#define BQ2562X_INIT(inst)                                                                                  \
                                                                                                            \
	static const struct bq2562x_config bq2562x_config_##inst = {                                        \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                          \
		.ce_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, ce_gpios, {}),                                    \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {}),                                  \
	};                                                                                                  \
                                                                                                            \
	static struct bq2562x_data bq2562x_data_##inst = {                                                  \
		.constant_charge_current_max_ua = DT_INST_PROP(inst, constant_charge_current_max_microamp), \
		.constant_charge_voltage_max_uv = DT_INST_PROP(inst, constant_charge_voltage_max_microvolt),\
		.precharge_current_ua = DT_INST_PROP(inst, precharge_current_microamp),                     \
		.charge_term_current_ua = DT_INST_PROP(inst, charge_term_current_microamp),                 \
		.min_sys_voltage_uv = DT_INST_PROP(inst, ti_min_sys_voltage_microvolt),                     \
		.input_voltage_min_uv = DT_INST_PROP(inst, ti_input_voltage_limit_microvolt),               \
		.input_current_max_ua = DT_INST_PROP(inst, ti_input_current_limit_microamp),                \
		.thermal_regulation_threshold = DT_INST_PROP(inst, ti_thermal_regulation_threshold),        \
		.switching_converter_freq = DT_INST_PROP(inst, ti_switching_converter_freq),                \
		.switching_converter_strength = DT_INST_PROP(inst, ti_switching_converter_strength),        \
	};                                                                                                  \
                                                                                                            \
	DEVICE_DT_INST_DEFINE(inst, bq2562x_init, NULL, &bq2562x_data_##inst,                               \
			      &bq2562x_config_##inst, POST_KERNEL, CONFIG_CHARGER_INIT_PRIORITY,            \
			      &bq2562x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BQ2562X_INIT)
