/*
 * Copyright (c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_bq34110

#include <init.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/util.h>
#include <drivers/i2c.h>
#include <sys/__assert.h>
#include <sys/byteorder.h>
#include <drivers/sensor.h>
#include "bq34110.h"

static int bq34110_cmd_reg_read(struct bq34110_data *bq34110,
				Commands_t reg_addr, int16_t *val)
{
	int ret;
	uint8_t i2c_data[2];

	ret = i2c_write_read(bq34110->i2c, DT_INST_REG_ADDR(0), &reg_addr,
			     1, &i2c_data, 2);
	if (ret < 0) {
		LOG_ERR("Failed to read command register");
		return ret;
	}

	*val = ((i2c_data[1] << 8) | i2c_data[0]);

	return 0;
}

/**
 * @brief sensor value get
 *
 * @return -ENOTSUP for unsupported channels
 */
static int bq34110_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	float int_temp;
	int32_t avg_power;
	struct bq34110_data *bq34110 = dev->data;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_TEMP:
		int_temp = (bq34110->internal_temperature * 0.1);
		int_temp = int_temp - 273.15;
		val->val1 = (int32_t)int_temp;
		val->val2 = (int_temp - (int32_t)int_temp) * 1000000;
		break;

	case SENSOR_CHAN_GAUGE_VOLTAGE:
		val->val1 = (bq34110->voltage / 1000);
		val->val2 = ((bq34110->voltage % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_AVG_POWER:
		avg_power = (bq34110->avg_power * 10);
		val->val1 = (avg_power / 1000);
		val->val2 = ((avg_power % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_AVG_CURRENT:
		val->val1 = (bq34110->avg_current / 1000);
		val->val2 = ((bq34110->avg_current % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_STATE_OF_HEALTH:
		val->val1 = bq34110->state_of_health;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		val->val1 = bq34110->state_of_charge;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_GAUGE_TIME_TO_FULL:
		val->val1 = bq34110->time_to_full;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_GAUGE_TIME_TO_EMPTY:
		val->val1 = bq34110->time_to_empty;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
		val->val1 = (bq34110->full_charge_capacity / 1000);
		val->val2 = ((bq34110->full_charge_capacity % 1000) * 1000U);
		break;

	case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
		val->val1 = (bq34110->remaining_charge_capacity / 1000);
		val->val2 =
			((bq34110->remaining_charge_capacity % 1000) * 1000U);
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int bq34110_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	int ret = 0;
	struct bq34110_data *bq34110 = dev->data;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_TEMP:
		ret = bq34110_cmd_reg_read(bq34110, INTERNAL_TEMPERATURE,
					   &bq34110->internal_temperature);
		if (ret < 0) {
			LOG_ERR("Failed to read temperature");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_VOLTAGE:
		ret = bq34110_cmd_reg_read(bq34110, VOLTAGE,
					   &bq34110->voltage);
		if (ret < 0) {
			LOG_ERR("Failed to read voltage");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_AVG_POWER:
		ret = bq34110_cmd_reg_read(bq34110, AVERAGE_POWER,
					   &bq34110->avg_power);
		if (ret < 0) {
			LOG_ERR("Failed to read Average Power");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_AVG_CURRENT:
		ret = bq34110_cmd_reg_read(bq34110, CURRENT,
					   &bq34110->avg_current);
		if (ret < 0) {
			LOG_ERR("Failed to read Average Current");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_STATE_OF_HEALTH:
		ret = bq34110_cmd_reg_read(bq34110, STATE_OF_HEALTH,
					   &bq34110->state_of_health);
		if (ret < 0) {
			LOG_ERR("Failed to read State of Health");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		ret = bq34110_cmd_reg_read(bq34110, RELATIVE_STATE_OF_CHARGE,
					   &bq34110->state_of_charge);
		if (ret < 0) {
			LOG_ERR("Failed to read State of Charge");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_TIME_TO_EMPTY:
		ret = bq34110_cmd_reg_read(bq34110, TIME_TO_EMPTY,
					   &bq34110->time_to_empty);
		if (ret < 0) {
			LOG_ERR("Failed to read Time to empty");
			return -EIO;
		}

	case SENSOR_CHAN_GAUGE_TIME_TO_FULL:
		ret = bq34110_cmd_reg_read(bq34110, TIME_TO_FULL,
					   &bq34110->time_to_full);
		if (ret < 0) {
			LOG_ERR("Failed to read Time to Full");
			return -EIO;
		}

	case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
		ret = bq34110_cmd_reg_read(bq34110, FULL_CHARGE_CAPACITY,
					   &bq34110->full_charge_capacity);
		if (ret < 0) {
			LOG_ERR("Failed to read Full Charge Capacity");
			return -EIO;
		}
		break;

	case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
		ret = bq34110_cmd_reg_read(bq34110, REMAINING_CAPACITY,
					   &bq34110->remaining_charge_capacity);
		if (ret < 0) {
			LOG_ERR("Failed to read Remaining Charge Capacity");
			return -EIO;
		}
		break;

	default:
		LOG_ERR("default fetch");
		return -ENOTSUP;
	}

	return 0;
}

static int bq34110_ctrl_reg_write(struct bq34110_data *bq34110,
				  uint16_t subcommand)
{
	int ret;
	uint8_t i2c_write_data[3] = {0, 0, 0};

	i2c_write_data[0] = 0x00;
	i2c_write_data[1] = (uint8_t)(subcommand & 0x00FF);
	i2c_write_data[2] = (uint8_t)((subcommand >> 8) & 0x00FF);
	ret = i2c_write(bq34110->i2c, i2c_write_data, sizeof(i2c_write_data),
			DT_INST_REG_ADDR(0));
	if (ret < 0) {
		LOG_ERR("Failed to write Ctrl subcmd");
		return ret;
	}

	return 0;
}

static int bq34110_ctrl_reg_read(struct bq34110_data *bq34110,
				 uint16_t subcommand,
				 uint16_t *val, Commands_t read_addr)
{
	int ret;
	uint8_t i2c_write_data[3] = { 0, 0, 0}, read_reg;
	uint8_t read_data[2];

	i2c_write_data[0] = MANUFACTURER_ACCESS_CONTROL;
	i2c_write_data[1] = (uint8_t)(subcommand & 0x00FF);
	i2c_write_data[2] = (uint8_t)((subcommand >> 8) & 0x00FF);
	ret = i2c_write(bq34110->i2c, i2c_write_data, 3,
			DT_INST_REG_ADDR(0));
	if (ret < 0) {
		LOG_ERR("Failed to write\n");
		return ret;
	}

	k_sleep(K_MSEC(1));

	read_reg = read_addr;
	ret = i2c_write_read(bq34110->i2c, DT_INST_REG_ADDR(0), &read_reg,
			     1, &read_data, 2);
	if (ret < 0) {
		LOG_ERR("Failed to read write");
		return ret;
	}

	*val = ((read_data[1] << 8) | read_data[0]);

	return 0;
}

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
static int bq34110_device_ctrl(const struct device *dev,
			       uint32_t ctrl_command,
			       void *context, device_pm_cb cb, void *arg)
{


	return 0;
}
#endif

void floating_point_conversion(float subclass_data, uint8_t *Data)
{
	int exp = 0;
	uint8_t byte0, byte1, byte2;
	float mod_val, tmp_val;

	mod_val = subclass_data;

	if (subclass_data != 0) {
		if (subclass_data < 0) {
			mod_val *= -1.0f;
		}

		tmp_val = mod_val;
		tmp_val *= (float)((1.0 + pow(2.0, -25.0)));
		if (tmp_val < 0.5) {
			while (tmp_val < 0.5) {
				tmp_val *= 2;
				exp--;
			}
		} else {
			if (tmp_val >= 1.0) {
				while (tmp_val >= 1.0) {
					tmp_val /= 2.0f;
					exp++;
				}
			}
		}

		if (exp > 127) {
			exp = 127;
		} else {
			if (exp < -128) {
				exp = -128;
			}
		}

		tmp_val = (float)((pow(2.0, 8 - exp)) * mod_val - 128);
		byte2 = (uint8_t)tmp_val;
		tmp_val = 256 * (tmp_val - byte2);
		byte1 = (uint8_t) tmp_val;
		tmp_val = 256 * (tmp_val - byte1);
		byte0 = (uint8_t)tmp_val;
		if (subclass_data < 0.0) {
			byte2 |= 0x80;
		}
	} else {
		exp = 0;
		byte2 = 0;
		byte1 = 0;
		byte0 = 0;
	}

	Data[0] = exp + 128;
	Data[1] = byte2;
	Data[2] = byte1;
	Data[3] = byte0;
}

static int enter_CalibrationMode(struct bq34110_data *bq34110)
{
	int ret;
	uint16_t flag;

	do {
		ret = bq34110_ctrl_reg_write(bq34110, CAL_TOOGLE);
		if (ret < 0) {
			LOG_ERR("Failed to perform CAL_TOGGLE");
			return ret;
		}

		k_sleep(K_MSEC(1));

		ret = bq34110_ctrl_reg_read(bq34110, MANUFACTURING_STATUS,
					    &flag, MAC_DATA);
		if (ret < 0) {
			LOG_ERR("Failed to get manufacturing status");
			return ret;
		}

		if (!(flag & 0x8000)) {
			k_sleep(K_MSEC(10));
		}
	} while (!(flag & 0x8000));

	return 0;
}

static int exit_CalibrationMode(struct bq34110_data *bq34110)
{
	int ret;
	uint16_t flag;

	ret = bq34110_ctrl_reg_write(bq34110, CAL_TOOGLE);
	if (ret < 0) {
		LOG_ERR("Failed to perform CAL_TOGGLE");
		return ret;
	}

	k_sleep(K_MSEC(1));

	ret = bq34110_ctrl_reg_read(bq34110, MANUFACTURING_STATUS,
				    &flag, MAC_DATA);
	if (ret < 0) {
		LOG_ERR("Failed to get manufacturing status");
		return ret;
	}

	LOG_ERR("Exit Calibration Status: 0x%x", flag);

	return 0;
}

static int CC_Offset_Board_Offset_Calibration(struct bq34110_data *bq34110)
{
	int ret;
	uint16_t flag;

	ret = enter_CalibrationMode(bq34110);
	if (ret < 0) {
		LOG_ERR("Failed to enter into Calibration mode");
		return ret;
	}


	do {
		ret = bq34110_ctrl_reg_write(bq34110, BOARD_OFFSET);
		if (ret < 0) {
			LOG_ERR("Failed to set Board Offset");
			return ret;
		}

		k_sleep(K_MSEC(1));

		ret = bq34110_ctrl_reg_read(bq34110, CONTROL_STATUS, &flag,
					    CONTROL);
		if (ret < 0) {
			LOG_ERR("Failed to get Status flag");
			return ret;
		}

		if (!(flag & 0x0030)) {
			k_sleep(K_MSEC(10));
		}
	} while (!(flag & 0x0030));

	do {
		ret = bq34110_ctrl_reg_read(bq34110, CONTROL_STATUS, &flag,
					    CONTROL);
		if (ret < 0) {
			LOG_ERR("Failed to get Control Status\n");
			return ret;
		}

		k_sleep(K_MSEC(500));

	} while (flag);

	ret = exit_CalibrationMode(bq34110);
	if (ret < 0) {
		LOG_ERR("Failed to exit calibration mode");
		return ret;
	}

	return 0;
}

static int Obtain_Raw_Calibration_Data(struct bq34110_data *bq34110,
				       struct rawDataAvg *val)
{
	int ret;
	uint8_t loopCount = 0, samplesToAvg = 20;
	uint16_t counterNow = 0, counterPrev = 0;
	uint16_t rawVoltage = 0, rawTemperature = 0;
	int16_t rawCurrent = 0;
	int32_t analog_current_sum = 0, analog_voltage_sum = 0,
		analog_temperature_sum = 0;

	ret = enter_CalibrationMode(bq34110);
	if (ret < 0) {
		LOG_ERR("Failed to enter into Calibration mode");
		return ret;
	}

	do {
		counterPrev = counterNow;
		ret = bq34110_cmd_reg_read(bq34110, ANALOG_COUNT,
					   &counterNow);
		if (ret < 0) {
			LOG_ERR("Failed to read Analog Count");
			return ret;
		}

		if (counterNow != counterPrev) {
			ret = bq34110_cmd_reg_read(bq34110, RAW_CURRENT,
						   &rawCurrent);
			if (ret < 0) {
				LOG_ERR("Failed to read Raw Current");
				return ret;
			}

			ret = bq34110_cmd_reg_read(bq34110, RAW_VOLTAGE,
						   &rawVoltage);
			if (ret < 0) {
				LOG_ERR("Failed to read Raw Voltage");
				return ret;
			}

			ret = bq34110_cmd_reg_read(bq34110, RAW_INT_TEMP,
						   &rawTemperature);
			if (ret < 0) {
				LOG_ERR("Failed to read raw Temperature");
				return ret;
			}

			analog_current_sum += rawCurrent;
			analog_voltage_sum += rawVoltage;
			analog_temperature_sum += rawTemperature;
			loopCount++;
		}

		k_sleep(K_MSEC(200));

	} while (loopCount < samplesToAvg);

	val->raw_current_avg = analog_current_sum / samplesToAvg;
	val->raw_voltage_avg = analog_voltage_sum / samplesToAvg;
	val->raw_temperature_avg = analog_temperature_sum / samplesToAvg;

	ret = exit_CalibrationMode(bq34110);
	if (ret < 0) {
		LOG_ERR("Failed to exit calibration mode");
		return ret;
	}

	return 0;
}

static int updateDFparameter(struct bq34110_data *bq34110, uint16_t df_addr,
			     uint8_t data[], uint8_t datalength)
{
	int ret;
	uint8_t i2c_df_addr_data[3], i2c_df_write_data[datalength+1], i;
	uint8_t i2c_datasum_datalen[3];
	uint16_t datasum;

	i2c_df_addr_data[0] = 0x3E;
	i2c_df_addr_data[1] = (uint8_t)(df_addr & 0x00FF);
	i2c_df_addr_data[2] = (uint8_t)((df_addr >> 8) & 0x00FF);

	ret = i2c_write(bq34110->i2c, i2c_df_addr_data, 3,
			DT_INST_REG_ADDR(0));
	if (ret < 0) {
		LOG_ERR("Failed to write into Manufacturer Access Control");
		return ret;
	}

	i2c_df_write_data[0] = MAC_DATA;
	for (i = 1; i <= datalength; i++) {
		i2c_df_write_data[i] = data[i-1];
	}

	ret = i2c_write(bq34110->i2c, i2c_df_write_data, datalength + 1,
			DT_INST_REG_ADDR(0));
	if (ret < 0) {
		LOG_ERR("Failed to write MAC Data");
		return ret;
	}

	datasum = i2c_df_addr_data[1] + i2c_df_addr_data[2];
	for (i = 1; i <= datalength; i++) {
		datasum += i2c_df_write_data[i];
	}

	LOG_ERR("Datasum: 0x%x", datasum);
	LOG_ERR("1's complement: 0x%x", (uint8_t)~datasum);

	i2c_datasum_datalen[0] = MAC_DATA_SUM;
	i2c_datasum_datalen[1] = (uint8_t)~datasum;
	i2c_datasum_datalen[2] = 4 + datalength;
	ret = i2c_write(bq34110->i2c, i2c_datasum_datalen,
			3, DT_INST_REG_ADDR(0));
	if (ret < 0) {
		LOG_ERR("Failed to write MACDatasum");
		return ret;
	}

	return 0;
}

static int readDFParameter(struct bq34110_data *bq34110, uint16_t df_addr,
			   void *val, size_t datalength)
{
	int ret;
	uint8_t i2c_write_data[3] = { 0, 0, 0}, read_reg;

	i2c_write_data[0] = MANUFACTURER_ACCESS_CONTROL;
	i2c_write_data[1] = (uint8_t)(df_addr & 0x00FF);
	i2c_write_data[2] = (uint8_t)((df_addr >> 8) & 0x00FF);
	ret = i2c_write(bq34110->i2c, i2c_write_data, 3,
			DT_INST_REG_ADDR(0));
	if (ret < 0) {
		LOG_ERR("Failed to write");
		return ret;
	}

	k_sleep(K_MSEC(1));

	read_reg = MAC_DATA;
	LOG_ERR("Datalength: 0x%x", datalength);
	ret = i2c_write_read(bq34110->i2c, DT_INST_REG_ADDR(0), &read_reg,
			     1, val, datalength);
	if (ret < 0) {
		LOG_ERR("Failed to read write");
		return ret;
	}

	return 0;
}

static int current_calibration(struct bq34110_data *bq34110,
					struct rawDataAvg *val)
{
	int ret;
	uint8_t data[8], read_data[2];
	int16_t ccOffset;
	float ccGain, ccDelta;
	uint16_t known_current;
	int8_t boardOffset;

	LOG_ERR("Current Calibration function");

	ret = enter_CalibrationMode(bq34110);
	if (ret < 0) {
		LOG_ERR("Failed to enter into Calibration mode");
		return ret;
	}

	/* Calibration Current Begin */
	int16_t ext_coeff_3;

	ret = readDFParameter(bq34110, EXT_COEFF_3, &read_data, 2);
	if (ret < 0) {
		LOG_ERR("Failed to read ext_coeff_3");
		return ret;
	}

	ext_coeff_3 = (read_data[0] << 8 | read_data[1]);

	LOG_DBG("ext_coeff_3: %d", ext_coeff_3);

	ret = readDFParameter(bq34110, CC_OFFSET, &read_data, 2);
	if (ret < 0) {
		LOG_ERR("Failed to read CC_OFFSET");
		return ret;
	}
	ccOffset = (read_data[0] << 8 | read_data[1]);


	ret = readDFParameter(bq34110, BOARD_OFFSET, &boardOffset, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read CC_OFFSET");
		return ret;
	}

	known_current = CONFIG_KNOWN_LOAD_CURRENT;
	ccGain = (float)((80 - (ccOffset + boardOffset)) / 16);
	ccGain = (float)(known_current / ccGain);
	ccDelta = (float)(ccGain * 1193046.0f);


	memset(&data, 0, sizeof(data));
	floating_point_conversion(ccGain, data);
	floating_point_conversion(ccDelta, &data[4]);
	ret = updateDFparameter(bq34110, CC_GAIN, data, 8);
	if (ret < 0) {
		LOG_ERR("Failed to update CC Gain & CC Delta");
		return ret;
	}

	ret = exit_CalibrationMode(bq34110);
	if (ret < 0) {
		LOG_ERR("Failed to exit calibration mode");
		return ret;
	}

	return 0;
}

static int temperature_Calibration(struct bq34110_data *bq34110,
				   struct rawDataAvg *val)
{
	int ret;
	int8_t known_temperature, raw_temperature, temp_offset;

	ret = enter_CalibrationMode(bq34110);
	if (ret < 0) {
		LOG_ERR("Failed to enter into Calibration mode");
		return ret;
	}

	known_temperature = CONFIG_BQ34110_KNOWN_TEMP;
	raw_temperature = (((val->raw_temperature_avg) * 0.1) - 273.15);
	temp_offset = known_temperature - raw_temperature;
	ret = updateDFparameter(bq34110, INT_TEMP_OFFSET, &temp_offset, 1);
	if (ret < 0) {
		LOG_ERR("Failed to update INT TEMP OFFSET");
		return ret;
	}

	temp_offset = 0;
	ret = readDFParameter(bq34110, INT_TEMP_OFFSET, &temp_offset, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read INT TEMP OFFSET");
		return ret;
	}

	ret = exit_CalibrationMode(bq34110);
	if (ret < 0) {
		LOG_ERR("Failed to exit calibration mode");
		return ret;
	}

	return 0;
}

static int voltage_Calibration(struct bq34110_data *bq34110,
				struct rawDataAvg *val)
{
	int ret;
	int8_t vOffset;
	uint16_t known_voltage;

	ret = enter_CalibrationMode(bq34110);
	if (ret < 0) {
		LOG_ERR("Failed to enter into Calibration mode");
		return ret;
	}

	/*Calibration Voltage Begin */
	known_voltage = CONFIG_BQ34110_KNOWN_VOLTAGE;
	vOffset = (known_voltage - (val->raw_voltage_avg));
	ret = updateDFparameter(bq34110, PACK_V_OFFSET, &vOffset, 1);
	if (ret < 0) {
		LOG_ERR("Failed to update vOffset");
		return ret;
	}

	ret = exit_CalibrationMode(bq34110);
	if (ret < 0) {
		LOG_ERR("Failed to exit calibration mode");
		return ret;
	}

	return 0;
}

/**
 * @brief initialise the fuel gauge
 *
 * @return 0 for success
 */
static int bq34110_gauge_init(const struct device *dev)
{
	int ret;
	int16_t design_capacity;
	struct rawDataAvg rawData;
	uint8_t readdata[2], writedata[2];
	struct bq34110_data *bq34110 = dev->data;
	const struct bq34110_config *const config = dev->config;
	uint16_t design_voltage, device_type, taper_current, taper_voltage;

	bq34110->i2c = device_get_binding(config->bus_name);
	if (bq34110->i2c == NULL) {
		LOG_ERR("Could not get pointer to %s device.",
			config->bus_name);
			return -EINVAL;
	}

	ret = bq34110_ctrl_reg_read(bq34110, SUB_DEVICE_TYPE,
				    &device_type, MAC_DATA);
	if (ret < 0) {
		LOG_ERR("Failed to get device type");
		return -EIO;
	}

	if (device_type != BQ34110_DEVICE_ID) {
		LOG_ERR("Invalid Device");
		return -EINVAL;
	}

	memset(&writedata, 0, sizeof(writedata));
	design_voltage = config->design_voltage;
	writedata[0] = (design_voltage >> 8) & 0x00FF;
	writedata[1] = (design_voltage & 0xFF);

	/* Update Design Voltage */
	ret = updateDFparameter(bq34110, DESIGN_VOLTAGE, writedata, 2);
	if (ret < 0) {
		LOG_ERR("Failed to update design voltage");
		return ret;
	}

	k_sleep(K_MSEC(1));

	memset(&writedata, 0, sizeof(writedata));
	design_capacity = config->design_capacity;
	writedata[0] = (design_capacity >> 8) & 0x00FF;
	writedata[1] = (design_capacity & 0xFF);

	/* Update Design Capacity */
	ret = updateDFparameter(bq34110, DESIGN_CAPACITY_MAH, writedata, 2);
	if (ret < 0) {
		LOG_ERR("Failed to update Design Capacity");
		return ret;
	}

	k_sleep(K_MSEC(1));

	memset(&writedata, 0, sizeof(writedata));
	writedata[0] = 0x02;
	writedata[1] = 0x04;

	/* Select Internal Temperature Sensor */
	ret = updateDFparameter(bq34110, OPERATION_CONFIG_A, writedata, 2);
	if (ret < 0) {
		LOG_ERR("Failed to update Operation Config A");
		return ret;
	}

	memset(&writedata, 0, sizeof(writedata));
	taper_current = config->taper_current;
	writedata[0] = (taper_current >> 8) & 0x00FF;
	writedata[1] = (taper_current & 0xFF);

	/* Update Taper Current */
	ret = updateDFparameter(bq34110, TAPER_CURRENT, writedata, 2);
	if (ret < 0) {
		LOG_ERR("Failed to update Taper Current");
		return ret;
	}

	k_sleep(K_MSEC(1));

	memset(&writedata, 0, sizeof(writedata));
	taper_voltage = config->taper_voltage;
	writedata[0] = (taper_voltage >> 8) & 0x00FF;
	writedata[1] = (taper_voltage & 0xFF);

	/* Update Taper Voltage */
	ret = updateDFparameter(bq34110, TAPER_VOLTAGE, writedata, 2);
	if (ret < 0) {
		LOG_ERR("Failed to update Taper Voltage");
		return ret;
	}

	k_sleep(K_MSEC(1));

	/* Below function's are used to calibrate the device */
	/*
	ret = CC_Offset_Board_Offset_Calibration(bq34110);
	if (ret < 0) {
		LOG_ERR("Failed to Calibrate CC & Board Offset\n");
		return ret;
	}

	LOG_DBG("Columb Calibration && Board Offset done\n");

	ret = Obtain_Raw_Calibration_Data(bq34110, &rawData);
	if (ret < 0) {
		LOG_ERR("Failed to Calibrate Current\n");
		return ret;
	}

	LOG_DBG("Obtained Raw Calibration data done\n");

	ret = voltage_Calibration(bq34110, &rawData);
	if (ret < 0) {
		LOG_ERR("Failed to Calibrate Voltage\n");
		return ret;
	}

	LOG_DBG("Voltage Calibration done\n");

	ret = current_calibration(bq34110, &rawData);
	if (ret < 0) {
		LOG_ERR("Failed to Calibrate Current\n");
		return ret;
	}

	LOG_DBG("Current Calibration done\n");

	ret = temperature_Calibration(bq34110, &rawData);
	if (ret < 0) {
		LOG_ERR("Failed to Calibrate temperature\n");
		return ret;
	}

	LOG_DBG("Temperature Calibration done\n");
*/
	return 0;
}

static const struct sensor_driver_api bq34110_battery_driver_api = {
#ifdef CONFIG_BQ34110_TRIGGER
	.attr_set = bq34110_attr_set,
	.trigger_set = bq34110_trigger_set,
#endif
	.sample_fetch = bq34110_sample_fetch,
	.channel_get = bq34110_channel_get,
};

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
#define BQ34110_DEVICE_INIT(index)					       \
	DEVICE_DEFINE(bq34110_##index, DT_INST_LABEL(index),		       \
		      bq34110_gauge_init, bq34110_device_ctrl,		       \
		      &bq34110_driver_##index, &bq34110_config_##index,	       \
		      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,		       \
		      &bq34110_battery_driver_api);

#else
#define BQ34110_DEVICE_INIT(index)					       \
	DEVICE_AND_API_INIT(bq34110_##index, DT_INST_LABEL(index),	       \
			    &bq34110_gauge_init, &bq34110_driver_##index,      \
			    &bq34110_config_##index, POST_KERNEL,	       \
			    CONFIG_SENSOR_INIT_PRIORITY,		       \
			    &bq34110_battery_driver_api);
#endif

#define BQ34110_INIT(index)						       \
	static struct bq34110_data bq34110_driver_##index;		       \
									       \
	static const struct bq34110_config bq34110_config_##index = {	       \
		.bus_name = DT_INST_BUS_LABEL(index),			       \
		.design_voltage = DT_INST_PROP(index, design_voltage),	       \
		.design_capacity = DT_INST_PROP(index, design_capacity),       \
		.taper_current = DT_INST_PROP(index, taper_current),	       \
		.taper_voltage = DT_INST_PROP(index, taper_voltage),	       \
		.no_of_series_cells = DT_INST_PROP(index, no_of_series_cells), \
	};								       \
									       \
	BQ34110_DEVICE_INIT(index);

DT_INST_FOREACH_STATUS_OKAY(BQ34110_INIT)
