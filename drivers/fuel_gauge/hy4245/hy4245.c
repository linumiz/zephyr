/*
 * Copyright (c) 2025, Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ht_hy4245

#include <zephyr/kernel.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/drivers/fuel_gauge/hy4245.h>

LOG_MODULE_REGISTER(HY4245);

#define HY4245_CHIPID				0x4245

#define HY4245_CMD_CTRL				0x00
#define HY4245_CMD_TEMPERATURE			0x06
#define HY4245_CMD_VOLTAGE			0x08
#define HY4245_CMD_CURRENT			0x0c
#define HY4245_CMD_CAPACITY_REM			0x10
#define HY4245_CMD_CAPACITY_FULL		0x12
#define HY4245_CMD_AVG_CURRENT			0x14
#define HY4245_CMD_TIME_TO_EMPTY		0x16
#define HY4245_CMD_TIME_TO_FULL			0x18
#define HY4245_CMD_CHRG_VOLTAGE			0x30
#define HY4245_CMD_CHRG_CURRENT			0x32
#define HY4245_CMD_CAPACITY_FULL_AVAIL		0x78
#define HY4245_CMD_RELATIVE_STATE_OF_CHRG	0x2c

#define HY4245_SUBCMD_CTRL_STATUS 	0x00
#define HY4245_SUBCMD_CTRL_CHIPID	0x55
#define HY4245_SUBCMD_CTRL_CALIB_MODE	0x40
#define HY4245_SUBCMD_CTRL_RESET      0x41

#define HY4245_EXTCMD_SUBCLASS		0x3E
#define HY4245_EXTCMD_BLOCK		0x3F
#define HY4245_EXTCMD_BLKDATA		0x40
#define HY4245_EXTCMD_BLKDATA_CHECKSUM	0x60
#define HY4245_EXTCMD_BLKDATA_CTRL	0x61

#define HY4245_UNSEAL_KEY 		0x88, 0x42, 0x80, 0x28
#define HY4245_UNSEAL_KEY_ACTION 	0xFF, 0xFF, 0xFF, 0xFF

#define CTRL_STATUS_BCA		BIT(10) /* block data calibration routine state */
#define CTRL_STATUS_CSV		BIT(12) /* data/instruction flash checksum */
#define CTRL_STATUS_SS		BIT(13) /* seal/unseal */

#define HY4245_MAX_BYTES_PER_BLOCK 	32

struct hy4245_config {
	struct i2c_dt_spec i2c;
};

struct hy4245_data {
	struct k_mutex mutex;
};

static int hy4245_read16(const struct device *dev, uint8_t cmd, uint16_t *val)
{
	uint8_t buffer[2];
	const struct hy4245_config *cfg = dev->config;
	int ret;

	ret = i2c_burst_read_dt(&cfg->i2c, cmd, buffer, sizeof(buffer));
	if (ret != 0) {
		LOG_ERR("Unable to read register, error %d", ret);
		return ret;
	}

	*val= sys_get_le16(buffer);
	return 0;
}

static int hy4245_unseal_device(const struct device *dev)
{
	int ret;
	const struct hy4245_config *cfg = dev->config;
	uint8_t key[] = {HY4245_CMD_CTRL, HY4245_UNSEAL_KEY};
	uint8_t key1[] = {HY4245_CMD_CTRL, HY4245_UNSEAL_KEY_ACTION};

	ret = i2c_write_dt(&cfg->i2c, key, sizeof(key));
	if (ret) {
		return ret;
	}

	k_sleep(K_MSEC(1));
	ret = i2c_write_dt(&cfg->i2c, key1, sizeof(key1));
	if (ret) {
		return ret;
	}

	return 0;
}

static int hy4245_ctrl_status(const struct device *dev, uint16_t status)
{
	uint16_t resp;
	uint8_t cmd[3] = { HY4245_CMD_CTRL, HY4245_SUBCMD_CTRL_STATUS };
	const struct hy4245_config *cfg = dev->config;
	int ret;
	const uint8_t retry = 3;

	/* check control status SS */
	for (int i = 0; i < retry; i++) {
		ret = i2c_write_read_dt(&cfg->i2c, cmd, sizeof(cmd),
					&resp, sizeof(resp));
		if (ret != 0) {
			return ret;
		}

		if ((resp & status) != 0) {
			ret = resp;
			break;
		}
		ret = -EIO;
		k_sleep(K_MSEC(1));
	}

	return ret;
}

int hy4245_set_calibration_mode(const struct device *dev)
{
	uint8_t cmd[3] = { HY4245_CMD_CTRL, HY4245_SUBCMD_CTRL_CALIB_MODE };
	const struct hy4245_config *cfg = dev->config;
	struct hy4245_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);

	ret = i2c_write_dt(&cfg->i2c, cmd, sizeof(cmd));
	if (ret < 0) {
		goto err;
	}

	ret = hy4245_ctrl_status(dev, CTRL_STATUS_BCA);
	if(ret < 0) {
		goto err;
	}

	if ((ret & CTRL_STATUS_BCA) != CTRL_STATUS_BCA) {
		ret = -EIO;
	}

	ret = 0;
err:
	k_mutex_unlock(&data->mutex);
	return ret;
}

int hy4245_enable_flash_access(const struct device *dev)
{
	int ret;
	uint8_t resp;
	uint8_t cmd[2] = { HY4245_EXTCMD_BLKDATA_CTRL };
	const struct hy4245_config *cfg = dev->config;
	struct hy4245_data *data = dev->data;

	k_mutex_lock(&data->mutex, K_FOREVER);

	ret = hy4245_unseal_device(dev);
	if (ret != 0) {
		goto err;
	}

	/* enable access to Data Flash memory */
	ret = i2c_write_dt(&cfg->i2c, cmd, sizeof(cmd));
	if (ret != 0) {
		goto err;
	}

	ret = i2c_write_read_dt(&cfg->i2c, cmd, 1,
				&resp, sizeof(resp));
	if (ret  != 0 ||
	    resp != 0) {
		ret = -EIO;
	}

err:
	k_mutex_unlock(&data->mutex);
	return ret;
}

static int hy4245_set_flash_class_block(const struct device *dev,
					uint8_t subclass,
					uint8_t block)
{
	uint8_t cmd[2] = { HY4245_EXTCMD_SUBCLASS, subclass };
	const struct hy4245_config *cfg = dev->config;
	int ret;

	ret = i2c_write_dt(&cfg->i2c, cmd, sizeof(cmd));
	if (ret != 0) {
		return ret;
	}

	ret = hy4245_ctrl_status(dev, CTRL_STATUS_CSV);
	if(ret < 0) {
		return ret;
	}

	if ((ret & CTRL_STATUS_CSV) != CTRL_STATUS_CSV) {
		return -EIO;
	}

	/* as per data sheet 10msec delay */
	k_sleep(K_MSEC(10));

	cmd[0] = HY4245_EXTCMD_BLOCK;
	cmd[1] = block;

	ret = i2c_write_dt(&cfg->i2c, cmd, sizeof(cmd));
	if (ret != 0) {
		return ret;
	}

	ret = hy4245_ctrl_status(dev, CTRL_STATUS_CSV);
	if(ret < 0) {
		return ret;
	}

	if ((ret & CTRL_STATUS_CSV) != CTRL_STATUS_CSV) {
		return -EIO;
	}

	/* as per data sheet 10msec delay */
	k_sleep(K_MSEC(10));

	return 0;
}

static int hy4245_read_block_data(const struct device *dev,
				  uint8_t *data,
				  uint8_t count)
{
	uint8_t cmd[1] = { HY4245_EXTCMD_BLKDATA };
	const struct hy4245_config *cfg = dev->config;

	return i2c_write_read_dt(&cfg->i2c, cmd, sizeof(cmd),
				 data, count);
}

static int hy4245_write_block_data(const struct device *dev,
				   uint8_t *data,
				   uint8_t count)
{
	uint8_t cmd[HY4245_MAX_BYTES_PER_BLOCK + 1] = { HY4245_EXTCMD_BLKDATA };
	const struct hy4245_config *cfg = dev->config;

	memcpy(&cmd[1], data, count);
	return i2c_write_dt(&cfg->i2c, cmd, count + 1);
}

int hy4245_access_flash_data(const struct device *dev,
			     uint8_t subclass,
			     uint8_t block,
			     uint8_t *data,
			     uint8_t count,
			     bool is_read)
{
	int ret;
	uint8_t cmd[2] = {HY4245_EXTCMD_BLKDATA_CHECKSUM};
	const struct hy4245_config *cfg = dev->config;
	struct hy4245_data *drvdata = dev->data;
	uint8_t checksum = 0;
	uint8_t resp = 0;

	if (!data ||
	    count > HY4245_MAX_BYTES_PER_BLOCK) {
		return -EINVAL;
	}

	k_mutex_lock(&drvdata->mutex, K_FOREVER);
	ret = hy4245_set_flash_class_block(dev, subclass, block);
	if (ret != 0) {
		goto err;
	}

	if (is_read) {
		ret = hy4245_read_block_data(dev, data, count);

	} else {
		ret = hy4245_write_block_data(dev, data, count);
	}

	if (ret < 0) {
		LOG_ERR("flash read/write error %d", ret);
		goto err;
	}

	ret = hy4245_ctrl_status(dev, CTRL_STATUS_CSV);
	if(ret < 0) {
		goto err;
	}

	if ((ret & CTRL_STATUS_CSV) != CTRL_STATUS_CSV) {
		goto err;
	}

	/* as per data sheet 10msec delay */
	k_sleep(K_MSEC(10));

	for (int i = 0; i < count; i++) {
		checksum += data[i];
	}

	if (!is_read) {
		cmd[1] = checksum;
		ret = i2c_write_dt(&cfg->i2c, cmd, sizeof(cmd));
		if (ret < 0) {
			goto err;
		}
		ret = hy4245_ctrl_status(dev, CTRL_STATUS_CSV);
		if (ret < 0) {
			goto err;
		}

		if ((ret & CTRL_STATUS_CSV) != CTRL_STATUS_CSV) {
			goto err;
		}
	}
	ret = i2c_write_read_dt(&cfg->i2c, cmd, 1, &resp, sizeof(resp));
	if (ret < 0) {
		LOG_ERR("checksum read error %d", ret);
		goto err;
	}

	checksum = 0xFF - checksum;

	if (checksum != resp) {
		LOG_ERR("checksum not matched error %x-%x", checksum, resp);
		ret = -EILSEQ;
	}

err:
	k_mutex_unlock(&drvdata->mutex);
	return ret;
}

int hy4245_reset(const struct device *dev)
{
	uint8_t cmd[3] = {HY4245_CMD_CTRL, HY4245_SUBCMD_CTRL_RESET};
	const struct hy4245_config *cfg = dev->config;

	return i2c_write_dt(&cfg->i2c, cmd, sizeof(cmd));
}

static int hy4245_get_prop(const struct device *dev, fuel_gauge_prop_t prop,
			   union fuel_gauge_prop_val *val)
{
	int ret;
	uint16_t raw;
	struct hy4245_data *data = dev->data;

	k_mutex_lock(&data->mutex, K_FOREVER);

	switch (prop) {
	case FUEL_GAUGE_TEMPERATURE:
		ret = hy4245_read16(dev, HY4245_CMD_TEMPERATURE, &raw);
		val->temperature = raw;
		break;
	case FUEL_GAUGE_VOLTAGE:
		ret = hy4245_read16(dev, HY4245_CMD_VOLTAGE, &raw);
		val->voltage = raw * 1000;
		break;
	case FUEL_GAUGE_CURRENT:
		ret = hy4245_read16(dev, HY4245_CMD_CURRENT, &raw);
		val->current = (int16_t)raw * 1000;
		break;
	case FUEL_GAUGE_REMAINING_CAPACITY:
		ret = hy4245_read16(dev, HY4245_CMD_CAPACITY_REM, &raw);
		val->remaining_capacity = raw * 1000;
		break;
	case FUEL_GAUGE_FULL_CHARGE_CAPACITY:
		ret = hy4245_read16(dev, HY4245_CMD_CAPACITY_FULL, &raw);
		val->full_charge_capacity = raw * 1000;
		break;
	case FUEL_GAUGE_AVG_CURRENT:
		ret = hy4245_read16(dev, HY4245_CMD_AVG_CURRENT, &raw);
		val->avg_current = (int16_t)raw * 1000;
		break;
	case FUEL_GAUGE_RUNTIME_TO_EMPTY:
		ret = hy4245_read16(dev, HY4245_CMD_TIME_TO_EMPTY, &raw);
		val->runtime_to_empty = raw;
		break;
	case FUEL_GAUGE_RUNTIME_TO_FULL:
		ret = hy4245_read16(dev, HY4245_CMD_TIME_TO_FULL, &raw);
		val->runtime_to_full = raw;
		break;
	case FUEL_GAUGE_CHARGE_VOLTAGE:
		ret = hy4245_read16(dev, HY4245_CMD_CHRG_VOLTAGE, &raw);
		val->chg_voltage = raw * 1000;
		break;
	case FUEL_GAUGE_CHARGE_CURRENT:
		ret = hy4245_read16(dev, HY4245_CMD_CHRG_CURRENT, &raw);
		val->chg_current = raw * 1000;
		break;
	case FUEL_GAUGE_DESIGN_CAPACITY:
		ret = hy4245_read16(dev, HY4245_CMD_CAPACITY_FULL_AVAIL, &raw);
		val->design_cap = raw;
		break;
	case FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE:
		ret = hy4245_read16(dev, HY4245_CMD_RELATIVE_STATE_OF_CHRG, &raw);
		val->relative_state_of_charge = raw;
		break;
	default:
		ret = -ENOTSUP;
	}

	k_mutex_unlock(&data->mutex);
	return ret;
}

static int hy4245_init(const struct device *dev)
{
	int ret;
	const struct hy4245_config *cfg = dev->config;
	struct hy4245_data *data = dev->data;
	uint8_t cmd[3] = {HY4245_CMD_CTRL,  HY4245_SUBCMD_CTRL_CHIPID};
	uint16_t chip_id;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	ret = i2c_write_read_dt(&cfg->i2c, cmd, sizeof(cmd),
				&chip_id, sizeof(chip_id));
	if (ret != 0) {
		LOG_ERR("Unable to read register, error %d", ret);
		return ret;
	}

	if (chip_id != HY4245_CHIPID) {
		LOG_ERR("unknown chip id %x", chip_id);
		return -ENODEV;
	}

	k_mutex_init(&data->mutex);
	return 0;
}

static DEVICE_API(fuel_gauge, hy4245_driver_api) = {
	.get_property = &hy4245_get_prop,
};

#define HY4245_INIT(index)								\
                          								\
	static const struct hy4245_config hy4245_config_##index = {			\
		.i2c = I2C_DT_SPEC_INST_GET(index),					\
	};										\
											\
	static struct hy4245_data hy4245_data_##index;					\
											\
	DEVICE_DT_INST_DEFINE(index, &hy4245_init, NULL, &hy4245_data_##index,		\
			      &hy4245_config_##index, POST_KERNEL, 			\
			      CONFIG_FUEL_GAUGE_INIT_PRIORITY, &hy4245_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HY4245_INIT)
