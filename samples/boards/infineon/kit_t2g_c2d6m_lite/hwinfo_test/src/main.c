#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/printk.h>

static void print_hex(const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		printk("%02X", buf[i]);
	}
}

void main(void)
{
	uint8_t id1[16];
	uint8_t id2[16];
	uint32_t cause = 0;
	uint32_t supported = 0;
	ssize_t len1, len2;
	int ret;

	printk("IFX TRAVEO T2G hwinfo sample start\n");

	len1 = hwinfo_get_device_id(id1, sizeof(id1));
	if (len1 < 0) {
		printk("hwinfo_get_device_id() first call failed: %d\n", (int)len1);
		goto out;
	}

	len2 = hwinfo_get_device_id(id2, sizeof(id2));
	if (len2 < 0) {
		printk("hwinfo_get_device_id() second call failed: %d\n", (int)len2);
		goto out;
	}

	printk("Device ID #1 (%d bytes): ", (int)len1);
	print_hex(id1, len1);
	printk("\n");

	printk("Device ID #2 (%d bytes): ", (int)len2);
	print_hex(id2, len2);
	printk("\n");

	if (len1 != len2 || memcmp(id1, id2, len1) != 0) {
		printk("ERROR: Device ID is not stable across calls!\n");
	} else {
		printk("Device ID is stable across calls.\n");
	}
	ret = hwinfo_get_supported_reset_cause(&supported);
	if (ret == 0) {
		printk("0x%08x\n", supported);
	} 
	ret = hwinfo_get_reset_cause(&cause);
	if (ret == 0) {
		printk("Reset cause before clear: 0x%08X\n", cause);
	} else {
		printk("hwinfo_get_reset_cause() failed: %d\n", ret);
	}
	ret = hwinfo_clear_reset_cause();
	printk("hwinfo_clear_reset_cause(): %d\n", ret);
	ret = hwinfo_get_reset_cause(&cause);
	if (ret == 0) {
		printk("Reset cause after clear:  0x%08X\n", cause);
	} else {
		printk("hwinfo_get_reset_cause() after clear failed: %d\n", ret);
	}

out:
	printk("IFX TRAVEO T2G hwinfo sample done\n");
}
