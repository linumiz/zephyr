#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/pm/pm.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

static void set_mode_run(uint8_t state)
{
	switch (state) {
	case DL_SYSCTL_POWER_POLICY_RUN_SLEEP0:
		DL_SYSCTL_setPowerPolicyRUN0SLEEP0();
		break;
	case DL_SYSCTL_POWER_POLICY_RUN_SLEEP1:
		DL_SYSCTL_setPowerPolicyRUN1SLEEP1();
		break;
	case DL_SYSCTL_POWER_POLICY_RUN_SLEEP2:
		DL_SYSCTL_setPowerPolicyRUN2SLEEP2();
		break;
	}
}

static void set_mode_stop(uint8_t state)
{
	switch (state) {
	case DL_SYSCTL_POWER_POLICY_STOP0:
		DL_SYSCTL_setPowerPolicySTOP0();
		break;
	case DL_SYSCTL_POWER_POLICY_STOP1:
		DL_SYSCTL_setPowerPolicySTOP1();
		break;
	case DL_SYSCTL_POWER_POLICY_STOP2:
		DL_SYSCTL_setPowerPolicySTOP2();
		break;
	}
}

static void set_mode_standby(uint8_t state)
{
	switch (state) {
	case DL_SYSCTL_POWER_POLICY_STANDBY0:
		DL_SYSCTL_setPowerPolicySTANDBY0();
		break;
	case DL_SYSCTL_POWER_POLICY_STANDBY1:
		DL_SYSCTL_setPowerPolicySTANDBY1();
		break;
	}
}

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	switch (state) {
	case PM_STATE_RUNTIME_IDLE:
		set_mode_run(substate_id);
		break;
	case PM_STATE_SUSPEND_TO_IDLE:
		set_mode_stop(substate_id);
		break;
	case PM_STATE_STANDBY:
		set_mode_standby(substate_id);
		break;
	case PM_STATE_SOFT_OFF:
		DL_SYSCTL_setPowerPolicySHUTDOWN();
		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		return;
	}

	__WFI();
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	DL_SYSCTL_setPowerPolicyRUN0SLEEP0();
	irq_unlock(0);
}

static int ti_mspm0l2xxx_pm_init(void)
{
	int ret;
	uint32_t rst_cause;

	ret = hwinfo_get_reset_cause(&rst_cause);
	if (ret != 0) {
		return ret;
	}

	if (RESET_LOW_POWER_WAKE == rst_cause)
	{
		DL_SYSCTL_releaseShutdownIO();
	}

	return 0;
}
SYS_INIT(ti_mspm0l2xxx_pm_init, POST_KERNEL, 0);
