/*
 * Copyright (c) 2025 National Taiwan University Racing Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// glibc includes
#include <stdbool.h>
#include <stddef.h>
#include <time.h>

// zephyr includes
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

// canopennode includes
#include <CANopen.h>

#include <canopennode.h>

LOG_MODULE_REGISTER(canopen_time, CONFIG_CANOPEN_LOG_LEVEL);

/* macro ---------------------------------------------------------------------*/
/// @brief CANopen time reference in POSIX epoch time in seconds.
#define CO_SEC_REF 441763200UL

/// @brief Number of seconds in a day.
#define SEC_OF_DAY 86400

/* type ----------------------------------------------------------------------*/
struct canopen_time_ctx {
	struct k_work work;
	canopen_time_callback_t callback;
	void *user_data;
};

/* static function declaration -----------------------------------------------*/
/// @brief Callback function when receiving CANopen TIME object.
static void time_callback_pre(void *arg);

/// @brief Bottom half of @ref time_callback_pre.
static void time_work(struct k_work *work);

/* static variable -----------------------------------------------------------*/
static struct canopen_time_ctx g_ctx = {
	.work = Z_WORK_INITIALIZER(time_work),
};

/* function definition -------------------------------------------------------*/
int canopen_time_init()
{
	CO_TIME_initCallbackPre(CO->TIME, &g_ctx, time_callback_pre);

	return 0;
}

void canopen_time_set_callback(canopen_time_callback_t callback, void *user_data)
{
	g_ctx.callback = callback;
	g_ctx.user_data = user_data;
}

/* static function definition ------------------------------------------------*/
static void time_callback_pre(void *arg)
{
	struct canopen_time_ctx *ctx = arg;

	k_work_submit(&ctx->work);
}

static void time_work(struct k_work *work)
{
	struct canopen_time_ctx *ctx = CONTAINER_OF(work, struct canopen_time_ctx, work);

	CO_NMT_internalState_t NMTstate = CO_NMT_getInternalState(CO->NMT);
	if (NMTstate != CO_NMT_PRE_OPERATIONAL && NMTstate != CO_NMT_OPERATIONAL) {
		return;
	}

	CO_TIME_process(CO->TIME, true, 0);
	LOG_DBG("Received TIME object, days: %d, ms: %d", CO->TIME->days, CO->TIME->ms);

	time_t epoch = ((time_t)CO->TIME->days * SEC_OF_DAY + CO->TIME->ms / 1000) + CO_SEC_REF;

	if (ctx->callback) {
		ctx->callback(epoch, ctx->user_data);
	}
}
