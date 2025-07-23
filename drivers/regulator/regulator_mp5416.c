#define DT_COMPAT mps,mp5416

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mp5416, CONFIG_REGULATOR_LOG_LEVEL);

#define MP5416_REG_BUCK1 0x04
#define MP5416_REG_BUCK2 0x05
#define MP5416_REG_BUCK3 0x06
#define MP5416_REG_BUCK4 0x07
#define MP5416_REG_LDO1  0x08
#define MP5416_REG_LDO2  0x09
#define MP5416_REG_LDO3  0x0a
#define MP5416_REG_LDO4  0x0b
#define MP5416_REG_CTL2  0x02
#define MP5416_EN_MASK   BIT(7)
#define MP5416_VSET_MASK 0x7f
#define MP5416_REG_VENDOR_ID 0x11
#define MP5416_VSET_EN_MASK  0x80

#define MP5416_MIN_UV_BUCK1_3 600000
#define MP5416_STEP_UV_BUCK1_3 12500
#define MP5416_MIN_UV_OTHER   800000
#define MP5416_STEP_UV_OTHER  25000

struct mp5416_config {
        struct regulator_common_config common;
        struct i2c_dt_spec i2c;
        uint8_t vsel_reg;
};

struct mp5416_data {
        struct regulator_common_data data;
};

static  bool is_buck1_or_3(uint8_t reg)
{
        return reg == MP5416_REG_BUCK1 || reg == MP5416_REG_BUCK3;
}


static int mp5416_enable(const struct device *dev)
{
        const struct mp5416_config *cfg = dev->config;
        uint8_t val;

        int ret = i2c_reg_read_byte_dt(&cfg->i2c, cfg->vsel_reg, &val);
        if (ret < 0) {
                LOG_ERR("Failed to read VSEL reg");
                return ret;
        }

        val |= MP5416_EN_MASK;

        return i2c_reg_write_byte_dt(&cfg->i2c, cfg->vsel_reg, val);
}

static int mp5416_disable(const struct device *dev)
{
        const struct mp5416_config *cfg = dev->config;
        uint8_t val;

        int ret = i2c_reg_read_byte_dt(&cfg->i2c, cfg->vsel_reg, &val);
        if (ret < 0) {
                LOG_ERR("Failed to read VSEL reg");
                return ret;
        }

        val &= ~MP5416_EN_MASK;

        return i2c_reg_write_byte_dt(&cfg->i2c, cfg->vsel_reg, val);
}

static int mp5416_init(const struct device *dev)
{
        const struct mp5416_config *config = dev->config;
        uint8_t value;

        if(!i2c_is_ready_dt(&config->i2c)){
                LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
                return -ENODEV;
        }

        (void)i2c_reg_read_byte_dt(&config->i2c, MP5416_REG_VENDOR_ID, &value);
        LOG_DBG("id: 0x%x", value);
        return 0;
}
static int mp5416_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
    	const struct mp5416_config *cfg = dev->config;
    	uint8_t sel;
    	uint8_t reg_val;
    	int ret;

    	uint32_t base_uv = is_buck1_or_3(cfg->vsel_reg) ? MP5416_MIN_UV_BUCK1_3 : MP5416_MIN_UV_OTHER;
    	uint32_t step_uv = is_buck1_or_3(cfg->vsel_reg) ? MP5416_STEP_UV_BUCK1_3 : MP5416_STEP_UV_OTHER;

    	if (min_uv < base_uv)
        	min_uv = base_uv;

    	sel = (min_uv - base_uv) / step_uv;

    	if (base_uv + sel * step_uv > max_uv)
        	return -EINVAL;

    	ret = i2c_reg_read_byte_dt(&cfg->i2c, cfg->vsel_reg, &reg_val);
    	if (ret < 0)
        	return ret;

    	reg_val = (reg_val & MP5416_EN_MASK) | (sel & MP5416_VSET_MASK);

    	return i2c_reg_write_byte_dt(&cfg->i2c, cfg->vsel_reg, reg_val);
}

static int mp5416_get_voltage(const struct device *dev, int32_t *voltage_uv)
{
    	const struct mp5416_config *cfg = dev->config;
    	uint8_t reg_val;
    	int ret;

    	ret = i2c_reg_read_byte_dt(&cfg->i2c, cfg->vsel_reg, &reg_val);
    	if (ret < 0)
        	return ret;

    	uint32_t base_uv = is_buck1_or_3(cfg->vsel_reg) ? MP5416_MIN_UV_BUCK1_3 : MP5416_MIN_UV_OTHER;
    	uint32_t step_uv = is_buck1_or_3(cfg->vsel_reg) ? MP5416_STEP_UV_BUCK1_3 : MP5416_STEP_UV_OTHER;

    	*voltage_uv = base_uv + (reg_val & MP5416_VSET_MASK) * step_uv;
    	return 0;
}



static const struct regulator_driver_api mp5416_api = {
        .enable = mp5416_enable,
        .disable = mp5416_disable,
	.set_voltage = mp5416_set_voltage,
	.get_voltage = mp5416_get_voltage,
};

#define MP5416_DEFINE(node_id, id, reg)                                               \
        static const struct mp5416_config mp5416_config_##id = {                      \
                .common = REGULATOR_DT_COMMON_CONFIG_INIT(node_id),                   \
                .i2c = I2C_DT_SPEC_GET(DT_PARENT(node_id)),                           \
                .vsel_reg = reg,                                                      \
        };                                                                            \
        static struct mp5416_data mp5416_data_##id;                                   \
        DEVICE_DT_DEFINE(node_id, mp5416_init, NULL,                                  \
                &mp5416_data_##id, &mp5416_config_##id,                               \
                POST_KERNEL, CONFIG_REGULATOR_MP5416_INIT_PRIORITY, &mp5416_api)

#define MP5416_DEFINE_COND(inst, child, reg_macro)                                    \
        IF_ENABLED(                                                                   \
                DT_NODE_EXISTS(DT_INST_CHILD(inst, child)),                           \
                (MP5416_DEFINE(DT_INST_CHILD(inst, child), child##inst, reg_macro)))

#define MP5416_DEFINE_BUCK(inst, idx) \
        MP5416_DEFINE_COND(inst, buck##idx, MP5416_REG_BUCK##idx)

#define MP5416_DEFINE_LDO(inst, idx) \
        MP5416_DEFINE_COND(inst, ldo##idx, MP5416_REG_LDO##idx)


MP5416_DEFINE(DT_NODELABEL(buck1), buck1_manual, MP5416_REG_BUCK1);
MP5416_DEFINE(DT_NODELABEL(buck2), buck2_manual, MP5416_REG_BUCK2);
MP5416_DEFINE(DT_NODELABEL(buck3), buck3_manual, MP5416_REG_BUCK3);
MP5416_DEFINE(DT_NODELABEL(buck4), buck4_manual, MP5416_REG_BUCK4);
MP5416_DEFINE(DT_NODELABEL(ldo1), ldo1_manual, MP5416_REG_LDO1);
MP5416_DEFINE(DT_NODELABEL(ldo2), ldo2_manual, MP5416_REG_LDO2);
MP5416_DEFINE(DT_NODELABEL(ldo3), ldo3_manual, MP5416_REG_LDO3);
MP5416_DEFINE(DT_NODELABEL(ldo4), ldo4_manual, MP5416_REG_LDO4);


