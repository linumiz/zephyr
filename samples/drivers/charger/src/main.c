/*
 * Copyright (c) 2023 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/charger.h>
#include <sys/errno.h>

static const struct device *chgdev = DEVICE_DT_GET(DT_NODELABEL(charger));

static int print_all(void)
{
	int ret;
	charger_prop_t props[] = {
		CHARGER_PROP_ONLINE,
		CHARGER_PROP_CHARGE_TYPE,
		CHARGER_PROP_CHARGE_TYPE,
		CHARGER_PROP_CHARGE_TYPE,
		CHARGER_PROP_HEALTH,
		CHARGER_PROP_STATUS,
		CHARGER_PROP_USB_TYPE,
		CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA,
		CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV,
		CHARGER_PROP_PRECHARGE_CURRENT_UA,
		CHARGER_PROP_CHARGE_TERM_CURRENT_UA,
		CHARGER_PROP_BATTERY_VOLTAGE_NOW,
		CHARGER_PROP_BATTERY_CURRENT_NOW,
		CHARGER_PROP_INPUT_VOLTAGE_NOW,
		CHARGER_PROP_INPUT_CURRENT_NOW,
		CHARGER_PROP_INPUT_REGULATION_CURRENT_UA,
		CHARGER_PROP_INPUT_REGULATION_VOLTAGE_UV
	};
	union charger_propval all[ARRAY_SIZE(props)] = {0};

	ret = charger_get_props(chgdev, props, all, ARRAY_SIZE(props));
	if (ret < 0)
		return ret;

	printk("CHARGER_PROP_ONLINE: %d\n", all[0].online);
	printk("CHARGER_PROP_CHARGE_TYPE: %d\n", all[3].charge_type);
	printk("CHARGER_PROP_HEALTH: %d\n", all[4].health);
	printk("CHARGER_PROP_STATUS: %d\n", all[5].status);
	printk("CHARGER_PROP_USB_TYPE: %d\n", all[6].usb_type);
	printk("CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA: %d\n", all[7].const_charge_current_ua);
	printk("CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV: %d\n", all[8].const_charge_voltage_uv);
	printk("CHARGER_PROP_PRECHARGE_CURRENT_UA: %d\n", all[9].precharge_current_ua);
	printk("CHARGER_PROP_CHARGE_TERM_CURRENT_UA: %d\n", all[10].charge_term_current_ua);
	printk("CHARGER_PROP_BATTERY_VOLTAGE_NOW: %d\n", all[11].battery_voltage_now_uv);
	printk("CHARGER_PROP_BATTERY_CURRENT_NOW: %d\n", all[12].battery_current_now_ua);
	printk("CHARGER_PROP_VOLTAGE_NOW: %d\n", all[13].input_voltage_now_uv);
	printk("CHARGER_PROP_CURRENT_NOW: %d\n", all[14].input_current_now_ua);
	printk("CHARGER_PROP_INPUT_REGULATION_CURRENT_UA: %d\n", all[15].input_current_regulation_current_ua);
	printk("CHARGER_PROP_INPUT_REGULATION_VOLTAGE_UV: %d\n", all[16].input_voltage_regulation_voltage_uv);

	return 0;
}

static void set_all(void)
{
	int ret;
	charger_prop_t props[] = {
		CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA,
		CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV,
		CHARGER_PROP_PRECHARGE_CURRENT_UA,
		CHARGER_PROP_CHARGE_TERM_CURRENT_UA,
		CHARGER_PROP_INPUT_REGULATION_CURRENT_UA,
		CHARGER_PROP_INPUT_REGULATION_VOLTAGE_UV,
	};
	union charger_propval all[ARRAY_SIZE(props)] = {
		{ 1040000 },
		{ 4800000 },
		{ 20000   },
		{ 100000  },
		{ 2000000 },
		{ 16800000},
	};

	ret = charger_set_props(chgdev, props, all, ARRAY_SIZE(props));
	if (ret < 0)
		return;
}

static void status_cb(enum charger_status status)
{
	printk("%s status %d\n", __func__, status);
}

static void online_cb(enum charger_online online)
{
	printk("%s status %d\n", __func__, online);
}

int main(void)
{
	union charger_propval val;
	int ret;

	if (chgdev == NULL) {
		printk("Error: no charger device found.\n");
		return 0;
	}

	if (!device_is_ready(chgdev)) {
		printk("Error: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       chgdev->name);
		return 0;
	}

	printk("Found device \"%s\", getting charger data\n", chgdev->name);

#ifdef CONFIG_CHARGER_DISCHARGE_CURRENT_NOTIFICATIONS
	val.discharge_current_notification.current_ua =
		CONFIG_APP_DISCHARGE_CURRENT_NOTIFICATION_THRESHOLD_UA;
	val.discharge_current_notification.severity = CHARGER_SEVERITY_WARNING;
	val.discharge_current_notification.duration_us =
		CONFIG_APP_DISCHARGE_CURRENT_NOTIFICATION_DURATION_US;

	ret = charger_set_prop(chgdev, CHARGER_PROP_DISCHARGE_CURRENT_NOTIFICATION, &val);
	if (ret < 0) {
		return ret;
	}
#endif

#ifdef CONFIG_CHARGER_SYSTEM_VOLTAGE_NOTIFICATIONS
	val.system_voltage_notification =
		CONFIG_APP_SYSTEM_VOLTAGE_NOTIFICATION_THRESHOLD_UV;

	ret = charger_set_prop(chgdev, CHARGER_PROP_SYSTEM_VOLTAGE_NOTIFICATION_UV, &val);
	if (ret < 0) {
		return ret;
	}
#endif

	union charger_propval stat;
	stat.status_notification = status_cb;

	union charger_propval online;
	online.online_notification = online_cb;

	charger_set_prop(chgdev, CHARGER_PROP_STATUS_NOTIFICATION, &stat);
	charger_set_prop(chgdev, CHARGER_PROP_ONLINE_NOTIFICATION, &online);

		ret = charger_charge_enable(chgdev, true);
		if (ret == -ENOTSUP) {
			printk("Enabling charge not supported, assuming auto charge enable\n");
		} else if (ret < 0) {
			return ret;
		}


	while (1) {
	//	k_sleep(K_FOREVER);
		print_all();
		//set_all();
		//print_all();
		//k_sleep(K_FOREVER);
		/* Poll until external power is presented to the charger */
/*
		do {
			ret = charger_get_prop(chgdev, CHARGER_PROP_ONLINE, &val);
			if (ret < 0) {
				return ret;
			}

			k_msleep(100);
		} while (val.online == CHARGER_ONLINE_OFFLINE);
*/

		val.status = CHARGER_STATUS_CHARGING;

		k_msleep(5000);

		ret = charger_get_prop(chgdev, CHARGER_PROP_STATUS, &val);
		if (ret < 0) {
			return ret;
		}

		switch (val.status) {
		case CHARGER_STATUS_CHARGING:
			printk("Charging in progress...\n");

			ret = charger_get_prop(chgdev, CHARGER_PROP_CHARGE_TYPE, &val);
			if (ret < 0) {
				return ret;
			}

			printk("Device \"%s\" charge type is %d\n", chgdev->name, val.charge_type);
			break;
		case CHARGER_STATUS_NOT_CHARGING:
			printk("Charging halted...\n");

			ret = charger_get_prop(chgdev, CHARGER_PROP_HEALTH, &val);
			if (ret < 0) {
				return ret;
			}

			printk("Device \"%s\" health is %d\n", chgdev->name, val.health);
			break;
		case CHARGER_STATUS_FULL:
			printk("Charging complete!");
			break;
		case CHARGER_STATUS_DISCHARGING:
			printk("External power removed, discharging\n");

			ret = charger_get_prop(chgdev, CHARGER_PROP_ONLINE, &val);
			if (ret < 0) {
				return ret;
			}
			break;
		default:
			return -EIO;
		}

		k_msleep(500);
	}
}
