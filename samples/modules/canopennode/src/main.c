#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <canopennode.h>

#define LOG_LEVEL CONFIG_CANOPEN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

#define CAN_INTERFACE DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus))
#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
                      DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
                             CONFIG_CAN_DEFAULT_BITRATE)) / 1000)
#define NMT_CONTROL                                                                                                    \
    CO_NMT_STARTUP_TO_OPERATIONAL                                                                                      \
    | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME        500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK        false
#define OD_STATUS_BITS       NULL

const struct device *dev = CAN_INTERFACE;
static bool canopen_initialized = false;

static void cleanup_can_filters(void)
{
    if (CO->CANmodule != NULL && CO->CANmodule->can_dev != NULL) {
        LOG_INF("Cleaning up existing CAN filters");
        for (int i = 0; i < CO->CANmodule->rx_size; i++) {
            if (CO->CANmodule->rx_array[i].filter_id != -ENOSPC) {
                can_remove_rx_filter(CO->CANmodule->can_dev,
                                   CO->CANmodule->rx_array[i].filter_id);
                CO->CANmodule->rx_array[i].filter_id = -ENOSPC;
                LOG_DBG("Removed filter %d", i);
            }
        }
    }
}

static int initialize_canopen_stack(void)
{
    CO_ReturnError_t err;
    uint32_t errInfo = 0;

    if (canopen_initialized) {
        LOG_INF("CANopen stack already initialized");
        return 0;
    }


    CO->CANmodule->CANnormal = false;
    err = CO_CANinit(CO, dev, CAN_BITRATE);
    if (err != CO_ERROR_NO) {
        LOG_ERR("CO_CANinit failed: %d", err);
        return -EIO;
    }

    LOG_INF("Initializing CANopen with node ID: %d", CONFIG_CANOPENNODE_NODE_ID);
    err = CO_CANopenInit(CO, NULL, NULL, OD, OD_STATUS_BITS,
                        NMT_CONTROL, FIRST_HB_TIME,
                        SDO_SRV_TIMEOUT_TIME, SDO_CLI_TIMEOUT_TIME,
                        SDO_CLI_BLOCK, CONFIG_CANOPENNODE_NODE_ID, &errInfo);

    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        LOG_ERR("CO_CANopenInit failed: %d, errInfo: 0x%X", err, errInfo);
        return -EIO;
    }

    err = CO_CANopenInitPDO(CO, CO->em, OD, CONFIG_CANOPENNODE_NODE_ID, &errInfo);
    if (err != CO_ERROR_NO) {
        LOG_ERR("CO_CANopenInitPDO failed: %d, errInfo: 0x%X", err, errInfo);
        return -EIO;
    }

    CO_CANsetNormalMode(CO->CANmodule);

    canopen_initialized = true;
    LOG_INF("CANopen stack initialized successfully");
    return 0;
}

int main(void)
{
    CO_ReturnError_t err;
    uint32_t heapMemoryUsed;

    if (!device_is_ready(dev)) {
        LOG_ERR("CAN interface not ready");
        return 0;
    }

    LOG_INF("CAN device is ready");
    CO = CO_new(NULL, &heapMemoryUsed);
    if (CO == NULL) {
        LOG_ERR("Failed to allocate CANopen objects");
        return 0;
    }

    LOG_INF("Allocated %u bytes for CANopen objects", heapMemoryUsed);

    LOG_INF("CANopenNode - Starting main loop");

    while (1) {
        CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
        int64_t last_time = k_uptime_get();

        if (initialize_canopen_stack() != 0) {
            LOG_ERR("Failed to initialize CANopen stack, retrying...");
            canopen_initialized = false;
            k_sleep(K_MSEC(2000));
            continue;
        }

        LOG_INF("Entering CANopen processing loop");

        last_time = k_uptime_get();
        int consecutive_timeouts = 0;

        while (reset == CO_RESET_NOT) {
            int64_t current_time = k_uptime_get();
            uint32_t timeDifference_us = (current_time - last_time) * 1000;
            last_time = current_time;

            reset = CO_process(CO, false, timeDifference_us, NULL);

            consecutive_timeouts = 0;

            k_sleep(K_MSEC(10));

            if (consecutive_timeouts > 100) {
                LOG_ERR("Too many consecutive timeouts, forcing reset");
                reset = CO_RESET_COMM;
                break;
            }
        }

        LOG_INF("CANopen reset requested: %d", reset);

        if (reset == CO_RESET_APP) {
            LOG_INF("Application reset requested");
            break;
        } else if (reset == CO_RESET_COMM) {
            LOG_INF("Communication reset requested - reinitializing");
            canopen_initialized = false;
            cleanup_can_filters();
            k_sleep(K_MSEC(100));
        }
    }

    LOG_INF("Application exiting");
    return 0;
}
