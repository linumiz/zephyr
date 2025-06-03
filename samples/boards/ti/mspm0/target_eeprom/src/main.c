/*
 * Copyright (c) 2024 Open Pixel Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/target/eeprom.h>

static const struct device *eeprom = DEVICE_DT_GET(DT_NODELABEL(eeprom0));
static struct k_timer reboot;

static void reboot_cb(struct k_timer *reboot)
{
	printk("reboot\n");
}

int main(void)
{
	k_timer_init(&reboot, reboot_cb, NULL);
	//k_timer_start(&reboot, K_SECONDS(5), K_SECONDS(5));

	printk("i2c target sample\n");

	if (!device_is_ready(eeprom)) {
		printk("eeprom device not ready\n");
		return 0;
	}

	if (i2c_target_driver_register(eeprom) < 0) {
		printk("Failed to register i2c target driver\n");
		return 0;
	}

	printk("i2c target driver registered\n");

	while (1) {
		printk("main %llu\n", k_uptime_get());
		k_sleep(K_SECONDS(1));
		//k_sleep(K_FOREVER);
	}

	if (i2c_target_driver_unregister(eeprom) < 0) {
		printk("Failed to unregister i2c target driver\n");
		return 0;
	}

	printk("i2c target driver unregistered\n");

	return 0;
}
