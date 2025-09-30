/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2025 National Taiwan University Racing Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <canopennode.h>

// glibc includes
#include <stdbool.h>
#include <stddef.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/init.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/util.h>

// canopennode includes
#include <CANopen.h>
#include <storage/CO_storage.h>
#include "OD.h"

LOG_MODULE_REGISTER(canopennode, CONFIG_CANOPEN_LOG_LEVEL);

/* macro ---------------------------------------------------------------------*/
#define NMT_CONTROL                                                                                \
	(COND_CODE_1(CONFIG_CANOPENNODE_NMT_STARTUP_TO_OPERATIONAL, (CO_NMT_STARTUP_TO_OPERATIONAL), (0)) |                                             \
		 COND_CODE_1(CONFIG_CANOPENNODE_NMT_ERR_ON_BUSOFF_HB, (CO_NMT_ERR_ON_BUSOFF_HB), (0)) |                                      \
			      (CONFIG_CANOPENNODE_NMT_ERR_ON_ERR_REG_MASK                          \
				       ? CO_NMT_ERR_ON_ERR_REG |                                   \
						 CONFIG_CANOPENNODE_NMT_ERR_ON_ERR_REG_MASK        \
				       : 0) |                                                      \
			      COND_CODE_1(CONFIG_CANOPENNODE_NMT_ERR_TO_STOPPED, (CO_NMT_ERR_TO_STOPPED), (0)) |                           \
					   COND_CODE_1(CONFIG_CANOPENNODE_NMT_ERR_FREE_TO_OPERATIONAL, (CO_NMT_ERR_FREE_TO_OPERATIONAL), (0)))
/* type ----------------------------------------------------------------------*/
struct canopen_ctx {
	uint8_t node_id;
	uint16_t bitrate;

	k_tid_t mainline_tid;
	k_tid_t sync_tid;

	struct k_thread mainline_thread;
	struct k_thread sync_thread;
};

/* static function declaration -----------------------------------------------*/
static int canopen_init(struct canopen_ctx *ctx);
static int canopen_reset_communication_impl(struct canopen_ctx *ctx);
static void canopen_start_threads(struct canopen_ctx *ctx);

static void mainline_thread(void *p1, void *p2, void *p3);
static void sync_thread(void *p1, void *p2, void *p3);

static void wakeup_mainline(void *object);
static void wakeup_sync(void *object);

static int od_new_init();
static int init();
static int thread_init();

/* exported variable ---------------------------------------------------------*/
CO_t *CO = NULL;

/* static variable -----------------------------------------------------------*/
static const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

#if CONFIG_CANOPENNODE_LEDS
static const struct gpio_dt_spec green_led =
	GPIO_DT_SPEC_GET(DT_CHOSEN(zephyr_canopen_green_led), gpios);
static const struct gpio_dt_spec red_led =
	GPIO_DT_SPEC_GET(DT_CHOSEN(zephyr_canopen_red_led), gpios);
#endif

static struct canopen_ctx g_ctx = {
	.node_id = CONFIG_CANOPENNODE_NODE_ID,
	.bitrate = CONFIG_CANOPENNODE_BITRATE,
	.mainline_tid = NULL,
	.sync_tid = NULL,
};

K_THREAD_STACK_DEFINE(mainline_thread_stack, CONFIG_CANOPENNODE_MAINLINE_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(sync_thread_stack, CONFIG_CANOPENNODE_SYNC_THREAD_STACK_SIZE);

// Call CO_new() before initialization of other application modules to to ensure
// is can be accessed even before canopen is initialized.
SYS_INIT(od_new_init, POST_KERNEL, 99);
SYS_INIT(init, APPLICATION, CONFIG_CANOPENNODE_INIT_PRIORITY);

// Start thread in the last step of initialization similar to static threads.
SYS_INIT(thread_init, APPLICATION, 99);

/* function definition -------------------------------------------------------*/
int canopen_reset_communication()
{
	return canopen_reset_communication_impl(&g_ctx);
}

/* static function declaration -----------------------------------------------*/
static int canopen_init(struct canopen_ctx *ctx)
{
	int err;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device not ready");
		return -ENODEV;
	}

#if CONFIG_CANOPENNODE_LEDS
	if (!gpio_is_ready_dt(&green_led)) {
		LOG_ERR("green LED device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&red_led)) {
		LOG_ERR("red LED device not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&green_led, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		LOG_ERR("Failed to configure green LED (err %d)", err);
		return err;
	}

	err = gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		LOG_ERR("Failed to configure red LED (err %d)", err);
		return err;
	}
#endif /* CONFIG_CANOPENNODE_LEDS */

	err = canopen_reset_communication_impl(ctx);
	if (err < 0) {
		LOG_ERR("failed to reset canopen communication (err %d)", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_CANOPENNODE_TIME)) {
		err = canopen_time_init();
		if (err < 0) {
			LOG_ERR("failed to initialize time (err %d)", err);
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_CANOPENNODE_STORAGE)) {
		err = canopen_storage_init();
		if (err < 0) {
			LOG_ERR("failed to initialize storage (err %d)", err);
			return err;
		}
	}

	return 0;
}

static int canopen_reset_communication_impl(struct canopen_ctx *ctx)
{
	int err;
	uint32_t error_info;
	uint16_t first_hb_time_ms;
	OD_entry_t *first_hb_time;
#if CONFIG_CANOPENNODE_LSS_SLAVE
	OD_entry_t *identity;
	uint32_t vendor_id;
	uint32_t product_code;
	uint32_t revision_number;
	uint32_t serial_number;
#endif

	bool thread_started = false;
	if (ctx->mainline_tid != NULL) {
		thread_started = true;

		/*
		 * Lock all mutecies before aborting threads to ensure they are not
		 * locked when aborted, with limited timeout to avoid deadlock.
		 */
		while (true) {
			if (k_mutex_lock(&CO->CANmodule->can_send_mutex, K_MSEC(100)) < 0) {
				/* does nothing */
			} else if (k_mutex_lock(&CO->CANmodule->od_mutex, K_MSEC(100)) < 0) {
				k_mutex_unlock(&CO->CANmodule->can_send_mutex);
			} else if (k_mutex_lock(&CO->CANmodule->emcy_mutex, K_MSEC(100)) < 0) {
				k_mutex_unlock(&CO->CANmodule->can_send_mutex);
				k_mutex_unlock(&CO->CANmodule->od_mutex);
			} else {
				break;
			}
		}

		k_thread_abort(ctx->mainline_tid);
		k_thread_abort(ctx->sync_tid);
		ctx->mainline_tid = NULL;
		ctx->sync_tid = NULL;

		k_mutex_unlock(&CO->CANmodule->can_send_mutex);
		k_mutex_unlock(&CO->CANmodule->od_mutex);
		k_mutex_unlock(&CO->CANmodule->emcy_mutex);
	}

	CO_CANmodule_disable(CO->CANmodule);
	err = CO_CANinit(CO, (void *)can_dev, ctx->bitrate);
	if (err != CO_ERROR_NO) {
		LOG_ERR("CO_CANinit failed (err %d)", err);
		return -EIO;
	}

#if CONFIG_CANOPENNODE_LSS_SLAVE
	if ((identity = OD_find(OD, OD_H1018_IDENTITY_OBJECT)) == NULL ||
	    OD_get_u32(identity, 1, &vendor_id, true) != ODR_OK ||
	    OD_get_u32(identity, 2, &product_code, true) != ODR_OK ||
	    OD_get_u32(identity, 3, &revision_number, true) != ODR_OK ||
	    OD_get_u32(identity, 4, &serial_number, true) != ODR_OK) {
		LOG_ERR("object dictionary error at entry 0x1018");
		return -EINVAL;
	}

	CO_LSS_address_t lssAddress = {
		.identity =
			{
				.vendorID = vendor_id,
				.productCode = product_code,
				.revisionNumber = revision_number,
				.serialNumber = serial_number,
			},
	};

	err = CO_LSSinit(CO, &lssAddress, &ctx->node_id, &ctx->bitrate);
	if (err != CO_ERROR_NO) {
		LOG_ERR("CO_LSSinit failed (err %d)", err);
		return -EIO;
	}

	CO_LSSslave_initCallbackPre(CO->LSSslave, ctx, wakeup_mainline);
#endif /* CONFIG_CANOPENNODE_LSS_SLAVE */

	if ((first_hb_time = OD_find(OD, OD_H1017_PRODUCER_HB_TIME)) == NULL ||
	    OD_get_u16(first_hb_time, 0, &first_hb_time_ms, true) != ODR_OK) {
		LOG_ERR("object dictionary error at entry 0x1017");
		return -EINVAL;
	}

	err = CO_CANopenInit(
		CO, NULL, NULL, OD, NULL, NMT_CONTROL, first_hb_time_ms,
		CONFIG_CANOPENNODE_SDO_TIMEOUT_TIME, CONFIG_CANOPENNODE_SDO_TIMEOUT_TIME,
		IS_ENABLED(CONFIG_CANOPENNODE_SDO_CLI_BLOCK), ctx->node_id, &error_info);
	if (err == CO_ERROR_OD_PARAMETERS) {
		LOG_ERR("object dictionary error at entry 0x%X", error_info);
		return -EINVAL;
	} else if (err != CO_ERROR_NO && (!(CO_CONFIG_LSS & CO_CONFIG_LSS_SLAVE) ||
					  err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)) {
		LOG_ERR("CO_CANopenInit failed (err %d)", err);
		return -EIO;
	}

	/* TIME callbackPre is processed separatedly in canopennode_time.c */
	CO_EM_initCallbackPre(CO->em, ctx, wakeup_mainline);
	CO_NMT_initCallbackPre(CO->NMT, ctx, wakeup_mainline);
#ifndef CONFIG_CANOPENNODE_HB_CONS_DISABLED
	CO_HBconsumer_initCallbackPre(CO->HBcons, ctx, wakeup_mainline);
#endif
	for (int i = 0; i < OD_CNT_SDO_SRV; i++) {
		CO_SDOserver_initCallbackPre(&CO->SDOserver[i], ctx, wakeup_mainline);
	}
#ifdef CONFIG_CANOPENNODE_SDO_CLI
	for (int i = 0; i < OD_CNT_SDO_CLI; i++) {
		CO_SDOclient_initCallbackPre(&CO->SDOclient[i], ctx, wakeup_mainline);
	}
#endif
#ifndef CONFIG_CANOPENNODE_SYNC_DISABLED
	CO_SYNC_initCallbackPre(CO->SYNC, ctx, wakeup_sync);
#endif

	err = CO_CANopenInitPDO(CO, CO->em, OD, ctx->node_id, &error_info);
	if (err == CO_ERROR_OD_PARAMETERS) {
		LOG_ERR("object dictionary error at entry 0x%X", error_info);
		return -EINVAL;
	} else if (err != CO_ERROR_NO) {
		LOG_ERR("CO_CANopenInitPDO failed (err %d)", err);
		return -EIO;
	}

	CO_RPDO_initCallbackPre(CO->RPDO, ctx, wakeup_sync);

	CO_CANsetNormalMode(CO->CANmodule);

	if (thread_started) {
		canopen_start_threads(ctx);
	}

	return 0;
}

static void canopen_start_threads(struct canopen_ctx *ctx)
{
	ctx->mainline_tid = k_thread_create(
		&ctx->mainline_thread, mainline_thread_stack,
		K_THREAD_STACK_SIZEOF(mainline_thread_stack), mainline_thread, NULL, NULL, NULL,
		CONFIG_CANOPENNODE_MAINLINE_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(ctx->mainline_tid, "canopen_mainline");

	ctx->sync_tid =
		k_thread_create(&ctx->sync_thread, sync_thread_stack,
				K_THREAD_STACK_SIZEOF(sync_thread_stack), sync_thread, NULL, NULL,
				NULL, CONFIG_CANOPENNODE_SYNC_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(ctx->sync_tid, "canopen_sync");
}

static void mainline_thread(void *p1, void *p2, void *p3)
{
	CO_NMT_reset_cmd_t reset;
	uint32_t start;       /* cycles */
	uint32_t stop;        /* cycles */
	uint32_t delta;       /* cycles */
	uint32_t elapsed = 0; /* microseconds */
	uint32_t next;        /* microseconds */

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		next = CONFIG_CANOPENNODE_MAINLINE_THREAD_PERIOD;
		start = k_cycle_get_32();

		reset = CO_process(CO, false, elapsed, &next);

#if CONFIG_CANOPENNODE_LEDS
#ifdef CONFIG_CANOPENNODE_LEDS_BICOLOR
		/* flavors red LED when both on */
		gpio_pin_set_dt(&red_led, CO_LED_GREEN(CO->LEDs, CO_LED_CANopen) &&
						  !CO_LED_RED(CO->LEDs, CO_LED_CANopen));
#else
		gpio_pin_set_dt(&green_led, CO_LED_GREEN(CO->LEDs, CO_LED_CANopen));
#endif
		gpio_pin_set_dt(&red_led, CO_LED_RED(CO->LEDs, CO_LED_CANopen));
#endif

#if CONFIG_CANOPENNODE_STORAGE
		canopen_storage_process();
#endif

		if (reset == CO_RESET_COMM) {
			LOG_INF("CANopen communication reset");
			canopen_reset_communication();
		} else if (reset == CO_RESET_APP) {
			LOG_INF("CANopen application reset");
			/* log panic to flush logs before reboot */
			log_panic();

			sys_reboot(SYS_REBOOT_COLD);
		}

		k_sleep(K_USEC(next));
		stop = k_cycle_get_32();
		delta = stop - start;
		elapsed = k_cyc_to_us_near32(delta);
	}
}

static void sync_thread(void *p1, void *p2, void *p3)
{
	uint32_t start;       /* cycles */
	uint32_t stop;        /* cycles */
	uint32_t delta;       /* cycles */
	uint32_t elapsed = 0; /* microseconds */
	uint32_t next;        /* microseconds */
#ifndef CONFIG_CANOPENNODE_SYNC_DISABLED
	CO_SYNC_status_t sync;
#endif

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		next = CONFIG_CANOPENNODE_SYNC_THREAD_PERIOD;
		start = k_cycle_get_32();

		CO_LOCK_OD(CO->CANmodule);

#ifndef CONFIG_CANOPENNODE_SYNC_DISABLED
		sync = CO_process_SYNC(CO, elapsed, &next);
#endif

#ifdef CONFIG_CANOPENNODE_PDO_SYNC
		CO_process_RPDO(CO, sync == CO_SYNC_RX_TX, elapsed, &next);
		CO_process_TPDO(CO, sync == CO_SYNC_RX_TX, elapsed, &next);
#else
		CO_process_RPDO(CO, false, elapsed, &next);
		CO_process_TPDO(CO, false, elapsed, &next);
#endif

		CO_UNLOCK_OD(CO->CANmodule);

		k_sleep(K_USEC(next));
		stop = k_cycle_get_32();
		delta = stop - start;
		elapsed = k_cyc_to_us_near32(delta);
	}
}

static void wakeup_mainline(void *object)
{
	struct canopen_ctx *ctx = object;

	if (ctx->mainline_tid != NULL) {
		k_wakeup(ctx->mainline_tid);
	}
}

static void wakeup_sync(void *object)
{
	struct canopen_ctx *ctx = object;

	if (ctx->sync_tid != NULL) {
		k_wakeup(ctx->sync_tid);
	}
}

static int od_new_init()
{
	CO = CO_new(NULL, NULL);
	return 0;
}

static int init()
{
	return canopen_init(&g_ctx);
}

static int thread_init()
{
	canopen_start_threads(&g_ctx);

	return 0;
}
