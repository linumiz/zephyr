/*
 * Copyright (c) 2024 Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/sys/reboot.h>
#include <ti/driverlib/m0p/dl_core.h>
#include <soc.h>

static int ti_mspm0g_init(void)
{
	/* Allow delay time to settle */
	delay_cycles(POWER_STARTUP_DELAY);

	/* Low Power Mode is configured to be SLEEP0 */
	DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

	return 0;
}

/* Overrides the weak ARM implementation */
void sys_arch_reboot(int type)
{
	switch (type)
	{
	case SYS_REBOOT_COLD:
		DL_SYSCTL_resetDevice(DL_SYSCTL_RESET_POR);
		break;
	default:
		NVIC_SystemReset();
		break;
	}
}

SYS_INIT(ti_mspm0g_init, PRE_KERNEL_1, 0);
