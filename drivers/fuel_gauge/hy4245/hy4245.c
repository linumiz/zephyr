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

LOG_MODULE_REGISTER(HY4245);

#define HY4245_CHIPID			0x4245

#define HY4245_CMD_CTRL			0x00
#define HY4245_CMD_FLASH		0x3e
#define HY4245_CMD_TEMPERATURE		0x06	
#define HY4245_CMD_VOLTAGE		0x08
#define HY4245_CMD_CURRENT		0x0c	
#define HY4245_CMD_CAPACITY_REM		0x10
#define HY4245_CMD_CAPACITY_FULL	0x12
#define HY4245_CMD_AVG_CURRENT		0x14
#define HY4245_CMD_TIME_TO_EMPTY	0x16
#define HY4245_CMD_TIME_TO_FULL		0x18
#define HY4245_CMD_CHRG_VOLTAGE		0x30
#define HY4245_CMD_CHRG_CURRENT		0x32
#define HY4245_CMD_CAPACITY_FULL_AVAIL	0x78

#define HY4245_SUBCMD_CTRL_STATUS	0x0
#define HY4245_SUBCMD_CTRL_CHIPID	0x55
#define HY4245_SUBCMD_CTRL_FLAG		0x77
#define HY4245_SUBCMD_CTRL_CFGA		0x98
#define HY4245_SUBCMD_CTRL_FLASH	0x03

struct hy4245_config {
	struct i2c_dt_spec i2c;
};

int hy4245_read16(const struct device *dev, uint8_t cmd, uint16_t *val)
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

static int hy4245_get_prop(const struct device *dev, fuel_gauge_prop_t prop,
			   union fuel_gauge_prop_val *val)
{
	int ret;
	uint16_t raw;

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
	default:
		ret = -ENOTSUP;
	}

	return ret;
}

static int hy4245_init(const struct device *dev)
{
	int ret;
	const struct hy4245_config *cfg;
	uint8_t cmd[4][3] = {{HY4245_CMD_CTRL,  HY4245_SUBCMD_CTRL_CHIPID},
		{HY4245_CMD_CTRL,  HY4245_SUBCMD_CTRL_STATUS},
		{HY4245_CMD_CTRL,  HY4245_SUBCMD_CTRL_FLAG},
		{HY4245_CMD_CTRL,  HY4245_SUBCMD_CTRL_CFGA},
	};
	uint8_t db[2] = {HY4245_CMD_FLASH,  HY4245_SUBCMD_CTRL_FLASH};
	uint16_t chip_id;

	cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

#if 1
	uint8_t hib[3] = {HY4245_CMD_CTRL, 0x12};
	ret = i2c_write_dt(&cfg->i2c, hib, sizeof(hib));

	k_msleep(100);
#endif

	for (int i = 0; i < 4; i++) {
		ret = i2c_write_read_dt(&cfg->i2c, cmd[i], sizeof(cmd[i]),
				&chip_id, sizeof(chip_id));
		if (ret != 0) {
			LOG_ERR("Unable to read register, error %d", ret);
			return ret;
		}

		printk("id %x\n", chip_id);
	}

	/* unseal */
	uint8_t db_key[] = {0x0, 0x88, 0x42, 0x80, 0x28};
	ret = i2c_write_dt(&cfg->i2c, db_key, sizeof(db_key));
	if (ret < 0)
		printk("FAIL %d\n", ret);

	uint8_t db_act[] = {0x0, 0xff, 0xff, 0xff, 0xff};
	ret = i2c_write_dt(&cfg->i2c, db_act, sizeof(db_act));
	if (ret < 0)
		printk("FAIL %d\n", ret);

	/* BCA */
	uint8_t db_bca[] = {0x0, 0x40, 0x0};
	ret = i2c_write_dt(&cfg->i2c, db_bca, sizeof(db_bca));
	if (ret < 0)
		printk("FAIL %d\n", ret);

	uint8_t db_stat[] = {0x0, 0x0, 0x0};
	
	ret = i2c_write_read_dt(&cfg->i2c, db_stat, sizeof(db_stat),
			&chip_id, sizeof(chip_id));
	if (ret < 0)
		printk("FAIL %d\n", ret);

	printk("BCA: %x\n", chip_id);

	/* block data control */
	uint8_t db_ctrl[] = {0x61, 0x00};
	ret = i2c_write_dt(&cfg->i2c, db_ctrl, sizeof(db_ctrl));
	if (ret < 0)
		printk("FAIL %d\n", ret);

	uint8_t db_ctrl_rd;
	ret = i2c_write_read_dt(&cfg->i2c, &db_ctrl, 1,
			&db_ctrl_rd, sizeof(db_ctrl_rd));
	printk("DB_CTRL_RD: %x\n", db_ctrl_rd);

	/* HY4245_SetDataFlashClassID */
	uint8_t db_class[] = {0x3e, 0x20};
	ret = i2c_write_dt(&cfg->i2c, db_class, sizeof(db_class));
	if (ret < 0)
		printk("FAIL %d %d\n", ret, __LINE__);

	uint8_t db_blk[] = {0x3f, 0x0};
	ret = i2c_write_dt(&cfg->i2c, db_blk, sizeof(db_blk));
	if (ret < 0)
		printk("FAIL %d %d\n", ret, __LINE__);

	for (int i = 0; i < 3; i++) {
		ret = i2c_write_read_dt(&cfg->i2c, db_stat, sizeof(db_stat),
				&chip_id, sizeof(chip_id));
		if (ret < 0)
			printk("FAIL %d %d\n", ret, __LINE__);

		printk("id: %x\n", chip_id);

		if(!(chip_id & BIT(12)))
			break;

		k_msleep(1000);
	}

	/* sleep 10ms */
	k_msleep(20);
	uint8_t db_rd[] = {0x40};
	uint8_t db_out[3] = {0};
	ret = i2c_write_read_dt(&cfg->i2c, db_rd, sizeof(db_rd),
			&db_out, sizeof(db_out));
	if (ret < 0)
		printk("FAIL %d\n", ret);

	printk("%x %x %x\n", db_out[0], db_out[1], db_out[2]);

	if (chip_id != HY4245_CHIPID) {
		LOG_ERR("unknown chip id %x", chip_id);
		return -ENODEV;
	}
	printk("unknown chip id %x\n", chip_id);
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
	DEVICE_DT_INST_DEFINE(index, &hy4245_init, NULL, NULL, &hy4245_config_##index,	\
			      POST_KERNEL, 99, &hy4245_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HY4245_INIT)
