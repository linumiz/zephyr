/*
 * Copyright (c) 2025 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT ti_mspm0_rtc

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include "rtc_utils.h"

#include <ti/driverlib/dl_rtc_common.h>

LOG_MODULE_REGISTER(rtc_ti_mspm0, CONFIG_RTC_LOG_LEVEL);

#define RTC_TI_ALARM_1		1
#define RTC_TI_ALARM_2		2

struct rtc_ti_mspm0_config {
	RTC_Regs *base;
	bool rtc_x;
	void (*irq_config_func)(const struct device *dev);
};

#if defined(CONFIG_RTC_ALARM)
struct rtc_ti_mspm0_alarm {
	uint16_t mask;
	bool is_pending;
	rtc_alarm_callback callback;
	void *user_data;
};
#endif

struct rtc_ti_mspm0_data {
	struct k_spinlock lock;
#if defined(CONFIG_RTC_ALARM)
	struct rtc_ti_mspm0_alarm rtc_alarm_1;
	struct rtc_ti_mspm0_alarm rtc_alarm_2;
#endif
};

static int rtc_ti_mspm0_set_time(const struct device *dev,
				 const struct rtc_time *timeptr)
{
	const struct rtc_ti_mspm0_config *cfg = dev->config;
	struct rtc_ti_mspm0_data *data = dev->data;

	if ((timeptr == NULL) || !rtc_utils_validate_rtc_time(timeptr, 0)) {
		return -EINVAL;
	}

	K_SPINLOCK(&data->lock) {
		DL_RTC_Common_setCalendarSecondsBinary(cfg->base, timeptr->tm_sec);
		DL_RTC_Common_setCalendarMinutesBinary(cfg->base, timeptr->tm_min);
		DL_RTC_Common_setCalendarHoursBinary(cfg->base, timeptr->tm_hour);
		DL_RTC_Common_setCalendarDayOfWeekBinary(cfg->base, timeptr->tm_wday);
		DL_RTC_Common_setCalendarDayOfMonthBinary(cfg->base, timeptr->tm_mday);
		DL_RTC_Common_setCalendarMonthBinary(cfg->base, timeptr->tm_mon);
		DL_RTC_Common_setCalendarYearBinary(cfg->base, timeptr->tm_year);
	}

	return 0;
}

static int rtc_ti_mspm0_get_time(const struct device *dev,
				 struct rtc_time *timeptr)
{
	const struct rtc_ti_mspm0_config *cfg = dev->config;
	struct rtc_ti_mspm0_data *data = dev->data;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	K_SPINLOCK(&data->lock) {
		timeptr->tm_sec  = DL_RTC_Common_getCalendarSecondsBinary(cfg->base);
		timeptr->tm_min  = DL_RTC_Common_getCalendarMinutesBinary(cfg->base);
		timeptr->tm_hour = DL_RTC_Common_getCalendarHoursBinary(cfg->base);
		timeptr->tm_mday = DL_RTC_Common_getCalendarDayOfMonthBinary(cfg->base);
		timeptr->tm_mon  = DL_RTC_Common_getCalendarMonthBinary(cfg->base);
		timeptr->tm_year = DL_RTC_Common_getCalendarYearBinary(cfg->base);
		timeptr->tm_wday = DL_RTC_Common_getCalendarDayOfWeekBinary(cfg->base);
	}

	return 0;
}

#if defined(CONFIG_RTC_ALARM)
static int rtc_ti_mspm0_alarm_get_supported_fields(const struct device *dev,
						   uint16_t id, uint16_t *mask)
{
	ARG_UNUSED(dev);

	if (id != RTC_TI_ALARM_1 && id != RTC_TI_ALARM_2) {
		return -EINVAL;
	}

	if (mask == NULL) {
		return -EINVAL;
	}

	*mask = (RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |
		 RTC_ALARM_TIME_MASK_WEEKDAY | RTC_ALARM_TIME_MASK_MONTHDAY);

	return 0;
}

static inline void rtc_ti_mspm0_set_alarm1(const struct device *dev,
					   uint16_t mask,
					   const struct rtc_time *timeptr)
{
	const struct rtc_ti_mspm0_config *cfg = dev->config;

	DL_RTC_Common_disableInterrupt(cfg->base, DL_RTC_COMMON_IIDX_ALARM1);
	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		cfg->base->A1MIN = 0;
		DL_RTC_Common_setAlarm1MinutesBinary(cfg->base, timeptr->tm_min);
		DL_RTC_Common_enableAlarm1MinutesBinary(cfg->base);
	}

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		DL_RTC_Common_setAlarm1HoursBinary(cfg->base, timeptr->tm_hour);
		DL_RTC_Common_enableAlarm1HoursBinary(cfg->base);
	}

	if (mask & RTC_ALARM_TIME_MASK_WEEKDAY) {
		DL_RTC_Common_setAlarm1DayOfWeekBinary(cfg->base, timeptr->tm_wday);
		DL_RTC_Common_enableAlarm1DayOfWeekBinary(cfg->base);
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		DL_RTC_Common_setAlarm1DayOfMonthBinary(cfg->base, timeptr->tm_mday);
		DL_RTC_Common_enableAlarm1DayOfMonthBinary(cfg->base);
	}

	DL_RTC_Common_enableInterrupt(cfg->base, DL_RTC_COMMON_IIDX_ALARM1);
}

static inline void rtc_ti_mspm0_set_alarm2(const struct device *dev,
					   uint16_t mask,
					   const struct rtc_time *timeptr)
{
	const struct rtc_ti_mspm0_config *cfg = dev->config;

	DL_RTC_Common_disableInterrupt(cfg->base, DL_RTC_COMMON_IIDX_ALARM2);
	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		cfg->base->A2MIN = 0;
		DL_RTC_Common_setAlarm2MinutesBinary(cfg->base, timeptr->tm_min);
		DL_RTC_Common_enableAlarm2MinutesBinary(cfg->base);
	}

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		DL_RTC_Common_setAlarm2HoursBinary(cfg->base, timeptr->tm_hour);
		DL_RTC_Common_enableAlarm2HoursBinary(cfg->base);
	}

	if (mask & RTC_ALARM_TIME_MASK_WEEKDAY) {
		DL_RTC_Common_setAlarm2DayOfWeekBinary(cfg->base, timeptr->tm_wday);
		DL_RTC_Common_enableAlarm2DayOfWeekBinary(cfg->base);
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		DL_RTC_Common_setAlarm2DayOfMonthBinary(cfg->base, timeptr->tm_mday);
		DL_RTC_Common_enableAlarm2DayOfMonthBinary(cfg->base);
	}

	DL_RTC_Common_enableInterrupt(cfg->base, DL_RTC_COMMON_IIDX_ALARM2);
}

static inline void rtc_ti_clear_alarm(const struct device *dev, uint16_t id)
{
	const struct rtc_ti_mspm0_config *cfg = dev->config;

	if (id == RTC_TI_ALARM_1) {
		cfg->base->A1MIN = 0x00;
		cfg->base->A1HOUR = 0x00;
		cfg->base->A1DAY = 0x00;
	} else {
		cfg->base->A2MIN = 0x00;
		cfg->base->A2HOUR = 0x00;
		cfg->base->A2DAY = 0x00;
	}
}

static int rtc_ti_mspm0_alarm_set_time(const struct device *dev, uint16_t id,
				       uint16_t mask,
				       const struct rtc_time *timeptr)
{
	struct rtc_ti_mspm0_data *data = dev->data;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	if (id != RTC_TI_ALARM_1 && id != RTC_TI_ALARM_2) {
		return -EINVAL;
	}

	K_SPINLOCK(&data->lock) {
		rtc_ti_clear_alarm(dev, id);
		if (id == RTC_TI_ALARM_1) {
			rtc_ti_mspm0_set_alarm1(dev, mask, timeptr);
			data->rtc_alarm_1.mask = mask;
			data->rtc_alarm_1.is_pending = false;
		} else {
			rtc_ti_mspm0_set_alarm2(dev, mask, timeptr);
			data->rtc_alarm_2.mask = mask;
			data->rtc_alarm_2.is_pending = false;
		}
	}

	return 0;
}

static int rtc_ti_mspm0_get_alarm1(const struct device *dev,
				   struct rtc_time *timeptr)
{
	uint16_t return_mask = 0;
	uint16_t alarm_mask = 0;
	const struct rtc_ti_mspm0_config *cfg = dev->config;
	struct rtc_ti_mspm0_data *data = dev->data;

	alarm_mask = data->rtc_alarm_1.mask;
	if (alarm_mask & RTC_ALARM_TIME_MASK_MINUTE) {
		timeptr->tm_min = DL_RTC_Common_getAlarm1MinutesBinary(cfg->base);
		return_mask |= RTC_ALARM_TIME_MASK_MINUTE;
	}

	if (alarm_mask & RTC_ALARM_TIME_MASK_HOUR) {
		timeptr->tm_hour = DL_RTC_Common_getAlarm1HoursBinary(cfg->base);
		return_mask |= RTC_ALARM_TIME_MASK_HOUR;
	}

	if (alarm_mask & RTC_ALARM_TIME_MASK_WEEKDAY) {
		timeptr->tm_wday = DL_RTC_Common_getAlarm1DayOfWeekBinary(cfg->base);
		return_mask |= RTC_ALARM_TIME_MASK_WEEKDAY;
	}

	if (alarm_mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		timeptr->tm_mday =  DL_RTC_Common_getAlarm1DayOfMonthBinary(cfg->base);
		return_mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
	}

	return return_mask;
}

static int rtc_ti_mspm0_get_alarm2(const struct device *dev, struct rtc_time *timeptr)
{
	uint16_t return_mask = 0;
	uint16_t alarm_mask = 0;
	const struct rtc_ti_mspm0_config *cfg = dev->config;
	struct rtc_ti_mspm0_data *data = dev->data;

	alarm_mask = data->rtc_alarm_2.mask;
	if (alarm_mask & RTC_ALARM_TIME_MASK_MINUTE) {
		timeptr->tm_min = DL_RTC_Common_getAlarm2MinutesBinary(cfg->base);
		return_mask |= RTC_ALARM_TIME_MASK_MINUTE;
	}

	if (alarm_mask & RTC_ALARM_TIME_MASK_HOUR) {
		timeptr->tm_hour = DL_RTC_Common_getAlarm2HoursBinary(cfg->base);
		return_mask |= RTC_ALARM_TIME_MASK_HOUR;
	}

	if (alarm_mask & RTC_ALARM_TIME_MASK_WEEKDAY) {
		timeptr->tm_wday = DL_RTC_Common_getAlarm2DayOfWeekBinary(cfg->base);
		return_mask |= RTC_ALARM_TIME_MASK_WEEKDAY;
	}

	if (alarm_mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		timeptr->tm_mday =  DL_RTC_Common_getAlarm2DayOfMonthBinary(cfg->base);
		return_mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
	}

	return return_mask;
}

static int rtc_ti_mspm0_alarm_get_time(const struct device *dev, uint16_t id,
				       uint16_t *mask, struct rtc_time *timeptr)
{
	struct rtc_ti_mspm0_data *data = dev->data;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	if (id != RTC_TI_ALARM_1 && id != RTC_TI_ALARM_2) {
		return -EINVAL;
	}

	K_SPINLOCK(&data->lock) {
		if (id == RTC_TI_ALARM_1) {
			*mask = rtc_ti_mspm0_get_alarm1(dev, timeptr);
		} else {
			*mask = rtc_ti_mspm0_get_alarm2(dev, timeptr);
		}
	}

	return 0;
}

static int rtc_ti_mspm0_alarm_set_callback(const struct device *dev, uint16_t id,
					   rtc_alarm_callback callback,
					   void *user_data)
{
	struct rtc_ti_mspm0_data *data = dev->data;
	struct rtc_ti_mspm0_alarm *tmp = NULL;

	if (callback == NULL) {
		return -EINVAL;
	}

	if (id != RTC_TI_ALARM_1 && id != RTC_TI_ALARM_2) {
		return -EINVAL;
	}

	K_SPINLOCK(&data->lock) {
		if (id == RTC_TI_ALARM_1) {
			tmp = &data->rtc_alarm_1;
		} else {
			tmp = &data->rtc_alarm_2;
		}

		tmp->callback = callback;
		tmp->user_data = user_data;
	}

	return 0;
}

static int rtc_ti_mspm0_alarm_is_pending(const struct device *dev, uint16_t id)
{
	int ret;
	struct rtc_ti_mspm0_alarm *alarm = NULL;
	struct rtc_ti_mspm0_data *data = dev->data;

	if (id != RTC_TI_ALARM_1 && id != RTC_TI_ALARM_2) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	switch (id) {
	case RTC_TI_ALARM_1:
		alarm = &data->rtc_alarm_1;
		break;
	case RTC_TI_ALARM_2:
		alarm = &data->rtc_alarm_2;
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	__disable_irq();
	ret = alarm->is_pending ? 1 : 0;
	alarm->is_pending = false;
	__enable_irq();

out:
	k_spin_unlock(&data->lock, key);
	return ret;
}
#endif

static void rtc_ti_mspm0_isr(const struct device *dev)
{
#if defined(CONFIG_RTC_ALARM)
	uint8_t id;
	uint32_t key = irq_lock();
	const struct rtc_ti_mspm0_config *cfg = dev->config;
	struct rtc_ti_mspm0_data *data = dev->data;
	struct rtc_ti_mspm0_alarm *alarm = NULL;

	switch (DL_RTC_Common_getPendingInterrupt(cfg->base)) {
	case DL_RTC_COMMON_IIDX_ALARM1:
		alarm = &data->rtc_alarm_1;
		id = RTC_TI_ALARM_1;
		break;
	case DL_RTC_COMMON_IIDX_ALARM2:
		alarm = &data->rtc_alarm_2;
		id = RTC_TI_ALARM_2;
		break;
	default:
		goto out;
	}

	alarm->is_pending = true;
	if (alarm->callback) {
		alarm->callback(dev, id, alarm->user_data);
	}

out:
	irq_unlock(key);
#endif
}

static int rtc_ti_mspm0_init(const struct device *dev)
{
	const struct rtc_ti_mspm0_config *cfg = dev->config;

	if (!cfg->rtc_x) {
		/* Enable power to RTC module */
		if (!DL_RTC_Common_isPowerEnabled(cfg->base)) {
			DL_RTC_Common_enablePower(cfg->base);
		}
	}

	DL_RTC_Common_enableClockControl(cfg->base);
	DL_RTC_Common_setClockFormat(cfg->base, DL_RTC_COMMON_FORMAT_BINARY);

	cfg->irq_config_func(dev);

	return 0;
}

static DEVICE_API(rtc, rtc_ti_mspm0_driver_api) = {
	.set_time		= rtc_ti_mspm0_set_time,
	.get_time		= rtc_ti_mspm0_get_time,
#if defined(CONFIG_RTC_ALARM)
	.alarm_set_time		= rtc_ti_mspm0_alarm_set_time,
	.alarm_get_time		= rtc_ti_mspm0_alarm_get_time,
	.alarm_is_pending	= rtc_ti_mspm0_alarm_is_pending,
	.alarm_set_callback	= rtc_ti_mspm0_alarm_set_callback,
	.alarm_get_supported_fields = rtc_ti_mspm0_alarm_get_supported_fields,
#endif /* CONFIG_RTC_ALARM */
};

#define RTC_TI_MSPM0_DEVICE_INIT(n)					     \
	static void ti_mspm0_config_irq_##n(const struct device *dev)	     \
	{								     \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),	     \
			    rtc_ti_mspm0_isr, DEVICE_DT_INST_GET(n), 0);     \
		irq_enable(DT_INST_IRQN(n));				     \
	}								     \
									     \
	static struct rtc_ti_mspm0_data rtc_data_##n;			     \
									     \
	static struct rtc_ti_mspm0_config rtc_config_##n = {		     \
	.base		 = (RTC_Regs *)DT_INST_REG_ADDR(n),		     \
	.rtc_x		 = DT_INST_PROP(n, ti_rtc_x),			     \
	.irq_config_func = ti_mspm0_config_irq_##n,			     \
	};								     \
									     \
DEVICE_DT_INST_DEFINE(n, &rtc_ti_mspm0_init, NULL, &rtc_data_##n,	     \
		      &rtc_config_##n, PRE_KERNEL_1,			     \
		      CONFIG_RTC_INIT_PRIORITY, &rtc_ti_mspm0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_TI_MSPM0_DEVICE_INIT);
