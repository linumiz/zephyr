#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define BUCK1_NODE DT_NODELABEL(buck1)
#define BUCK2_NODE DT_NODELABEL(buck2)
#define BUCK3_NODE DT_NODELABEL(buck3)
#define BUCK4_NODE DT_NODELABEL(buck4)
#define LDO1_NODE  DT_NODELABEL(ldo1)
#define LDO2_NODE  DT_NODELABEL(ldo2)
#define LDO3_NODE  DT_NODELABEL(ldo3)
#define LDO4_NODE  DT_NODELABEL(ldo4)

static void setup_regulator(const struct device *dev, const char *name, int32_t min_uv, int32_t max_uv)
{
    int ret;
    int32_t actual_uv;

    if (!device_is_ready(dev)) {
        LOG_ERR("%s not ready", name);
        return;
    }

    LOG_INF("%s is ready", name);

    ret = regulator_enable(dev);
    if (ret) {
        LOG_ERR("Failed to enable %s (err %d)", name, ret);
        return;
    }

    ret = regulator_set_voltage(dev, min_uv, max_uv);
    if (ret) {
        LOG_ERR("Failed to set voltage for %s (err %d)", name, ret);
        return;
    }

    ret = regulator_get_voltage(dev, &actual_uv);
    if (ret) {
        LOG_ERR("Failed to get voltage for %s (err %d)", name, ret);
        return;
    }

    LOG_INF("%s voltage set to %d uV", name, actual_uv);

    k_sleep(K_SECONDS(5));
}

void main(void)
{
    LOG_INF("MP5416 Regulator Initialization");

    setup_regulator(DEVICE_DT_GET(BUCK1_NODE), "BUCK1", 1200000, 1200000);
    setup_regulator(DEVICE_DT_GET(BUCK2_NODE), "BUCK2", 1500000, 1500000);
    setup_regulator(DEVICE_DT_GET(BUCK3_NODE), "BUCK3", 1800000, 1800000);
    setup_regulator(DEVICE_DT_GET(BUCK4_NODE), "BUCK4", 3300000, 3300000);

    setup_regulator(DEVICE_DT_GET(LDO1_NODE),  "LDO1", 1800000, 1800000);
    setup_regulator(DEVICE_DT_GET(LDO2_NODE),  "LDO2", 3300000, 3300000);
    setup_regulator(DEVICE_DT_GET(LDO3_NODE),  "LDO3", 3300000, 3300000);
    setup_regulator(DEVICE_DT_GET(LDO4_NODE),  "LDO4", 1100000, 1100000);

    LOG_INF("MP5416 configuration done");
}

