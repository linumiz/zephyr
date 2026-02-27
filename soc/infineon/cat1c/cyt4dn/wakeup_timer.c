/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/pm/pm.h>

#include <cy_rtc.h>
#include <cy_sysint.h>
#include <cy_sysclk.h>
#include <cy_syslib.h>

#include <soc.h>

#define SECS_PER_MIN	60U
#define SECS_PER_HOUR	3600U
#define SECS_PER_DAY	86400U
#define US_PER_SEC	1000000ULL

#define RTC_SYS_INT_IDX	srss_interrupt_backup_IRQn
#define RTC_CPU_INT_IDX	3U

static uint32_t entry_secs;

static uint32_t time_to_day_secs(const cy_stc_rtc_config_t *t)
{
	return (t->hour * SECS_PER_HOUR) +
	       (t->min * SECS_PER_MIN) +
	        t->sec;
}

static void day_secs_to_hms(uint32_t total, uint32_t *h,
			    uint32_t *m, uint32_t *s)
{
	total %= SECS_PER_DAY;
	*h = total / SECS_PER_HOUR;
	total %= SECS_PER_HOUR;
	*m = total / SECS_PER_MIN;
	*s = total % SECS_PER_MIN;
}

static void rtc_lptimer_isr(const void *arg)
{
	ARG_UNUSED(arg);
	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
}

void z_cms_lptim_hook_on_lpm_entry(uint64_t max_lpm_time_us)
{
	cy_stc_rtc_config_t now;
	cy_stc_rtc_alarm_t alarm;
	uint32_t timeout_sec;
	uint32_t alarm_total;
	uint32_t ah, am, as;

	Cy_RTC_GetDateAndTime(&now);
	entry_secs = time_to_day_secs(&now);

	timeout_sec = (uint32_t)((max_lpm_time_us + US_PER_SEC - 1U) /
				 US_PER_SEC);
	if (timeout_sec == 0U) {
		timeout_sec = 1U;
	}

	alarm_total = entry_secs + timeout_sec;
	day_secs_to_hms(alarm_total, &ah, &am, &as);

	(void)memset(&alarm, 0, sizeof(alarm));

	alarm.sec         = as;
	alarm.secEn       = CY_RTC_ALARM_ENABLE;
	alarm.min         = am;
	alarm.minEn       = CY_RTC_ALARM_ENABLE;
	alarm.hour        = ah;
	alarm.hourEn      = CY_RTC_ALARM_ENABLE;
	alarm.dayOfWeek   = CY_RTC_SUNDAY;
	alarm.dayOfWeekEn = CY_RTC_ALARM_DISABLE;
	alarm.date        = 1U;
	alarm.dateEn      = CY_RTC_ALARM_DISABLE;
	alarm.month       = CY_RTC_JANUARY;
	alarm.monthEn     = CY_RTC_ALARM_DISABLE;
	alarm.almEn       = CY_RTC_ALARM_ENABLE;

	Cy_RTC_SetAlarmDateAndTime(&alarm, CY_RTC_ALARM_1);

	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
	Cy_RTC_SetInterruptMask(CY_RTC_INTR_ALARM1);
}

uint64_t z_cms_lptim_hook_on_lpm_exit(void)
{
	cy_stc_rtc_config_t now;
	uint32_t exit_secs;
	uint32_t elapsed;

	Cy_RTC_GetDateAndTime(&now);
	exit_secs = time_to_day_secs(&now);

	if (exit_secs >= entry_secs) {
		elapsed = exit_secs - entry_secs;
	} else {
		elapsed = (SECS_PER_DAY - entry_secs) + exit_secs;
	}

	Cy_RTC_SetInterruptMask(0U);
	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);

	if (elapsed == 0U) {
		elapsed = 1U;
	}

	return (uint64_t)elapsed * US_PER_SEC;
}

static void ensure_clk_vars(void)
{
	extern uint32_t cy_AhbFreqHz;

	if (cy_AhbFreqHz == 0U) {
		SystemCoreClockUpdate();
	}

	if (cy_AhbFreqHz == 0U) {
		cy_AhbFreqHz = 8000000U;
	}
}

static int soc_lptimer_init(void)
{
	cy_stc_rtc_config_t init_time = {
		.sec       = 0U,
		.min       = 0U,
		.hour      = 12U,
		.amPm      = CY_RTC_AM,
		.hrFormat  = CY_RTC_24_HOURS,
		.dayOfWeek = CY_RTC_SATURDAY,
		.date      = 1U,
		.month     = CY_RTC_FEBRUARY,
		.year      = 25U,
	};

	/* MUST be first - prevents div/0 in PDL after XRES */
	ensure_clk_vars();

	/* Clear stale BACKUP state from previous boot */
	BACKUP_RTC_RW = 0U;
	Cy_RTC_SetInterruptMask(0U);
	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1 | CY_RTC_INTR_ALARM2);

	Cy_RTC_SelectClockSource(CY_SYSCLK_BAK_IN_ILO);
	Cy_RTC_Init(&init_time);

	enable_sys_int(RTC_SYS_INT_IDX,
		       RTC_CPU_INT_IDX,
		       rtc_lptimer_isr,
		       NULL);

	return 0;
}

SYS_INIT(soc_lptimer_init, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
