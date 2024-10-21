/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Linumiz
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_CS5490_H_
#define ZEPHYR_DRIVERS_SENSOR_CS5490_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

/* Commands */
#define READ_CMD		0b00000000
#define WRITE_CMD		0b01000000
#define PAGE_SELECT_CMD		0b10000000
#define INSTRUCTION_CMD		0b11000000

/* Pages */
#define CS5490_PAGE_0			0
#define CS5490_PAGE_16			16
#define CS5490_PAGE_17			17
#define CS5490_PAGE_18			18

/* Registers */
#define REG_CONFIG_0		0 /* In Page 0 */
#define REG_CONFIG_1		1
#define REG_CONFIG_2		0 /* In Page 1 */
#define REG_SERIAL_CTRL		7
#define REG_LOCK_CTRL		34
#define REG_INT_STATUS		23
#define REG_INT_MASK		3
#define REG_IOVR_DUR		4 /* In Page 17 */
#define REG_IOVR_LEVEL		5 /* In Page 17 */
#define REG_INST_CURRENT	2 /* In Page 16 */
#define REG_INST_VOLTAGE	3 /* In Page 16 */
#define REG_INST_POWER		4 /* In Page 16 */
#define REG_ACTIVE_POWER	5 /* In Page 16 */
#define REG_RMS_CURRENT		6
#define REG_RMS_VOLTAGE		7
#define REG_REACT_POWER		14
#define REG_PEAK_CURRENT	37
#define REG_PEAK_VOLTAGE	36
#define REG_APPARENT_POWER	20
#define REG_POWER_FACTOR	20
#define REG_TEMPERATURE		27 /* In Page 16 */

/* Interrupts */
#define CS5490_INT_DRDY_MASK	BIT(23)
#define CS5490_INT_VSWELL_MASK	BIT(16)
#define CS5490_INT_POR_MASK	BIT(14)
#define CS5490_INT_IOR_MASK	BIT(12)
#define CS5490_INT_VOR_MASK	BIT(10)
#define CS5490_INT_IOC_MASK	BIT(8)
#define CS5490_INT_VSAG_MASK	BIT(6)

#define CS5490_INT_CFG_MASK	GENMASK(0,4)
#define CS5490_MCLK		4.096

enum cs5490_interrupts {
	CS5490_INT_DRDY,	/* Data Ready */
	CS5490_INT_IOC,		/* Overcurrent */
	CS5490_INT_POR,		/* Power overflow */
	CS5490_INT_IOR,		/* Current overlfow */
	CS5490_INT_VOR,		/* Voltage overflow */
	CS5490_INT_VSAG,		/* Volatge sag */
	CS5490_INT_VSWELL,	/* Volatge swell */
	CS5490_INT_MAX,
};

struct cs5490_rx_frame {
	uint8_t data0;
	uint8_t data1;
	uint8_t data2;
}__packed;

struct cs5490 {
	float inst_current;
	float inst_voltage;
	float inst_active_power;
	float peak_current;
	float peak_voltage;
	float rms_current;
	float rms_voltage;
	float active_power;
	float reactive_power;
	float apparent_power;
	float power_factor;
};

struct cs5490_config {
	const struct device *dev;
	struct gpio_dt_spec int_gpio;
};

struct cs5490_data {
#if defined(CONFIG_CS5490_TRIGGER)
	const struct device *int_gpio;
	struct gpio_callback int_gpio_cb;
	sensor_trigger_handler_t handler[CS5490_INT_MAX];
	const struct sensor_trigger *trigger[CS5490_INT_MAX];
#if defined(CONFIG_CS5490_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(cs5490_stack, CONFIG_CS5490_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem trig_sem;
#else
	struct k_work work;
#endif /* CONFIG_CS5490_TRIGGER_OWN_THREAD */
#endif /* CONFIG_CS5490_TRIGGER */
	struct cs5490 sensor_data;
	struct k_sem uart_tx_sem;
	struct k_sem uart_rx_sem;
	struct k_sem lock;
};

static inline int cs5490_send(const struct device *dev,
			      uint8_t *send_data, size_t len)
{
	int rc;
	const struct cs5490_config *cfg = dev->config;
	struct cs5490_data *data = dev->data;

	if ((send_data == NULL) || (len == 0)) {
		return -EINVAL;
	}

	printk("send data len %u\n", len);
	rc = uart_tx(cfg->dev, send_data, len, 1000 * USEC_PER_MSEC);
	if (rc < 0) {
		return rc;
	}

	if (k_sem_take(&data->uart_tx_sem, K_MSEC(1500))) {
		return -ETIMEDOUT;
	}

	k_msleep(20);

	return rc;
}

static inline int cs5490_transceive(const struct device *dev,
				    uint8_t *send_data, size_t len,
				    uint8_t *recv_buf, size_t recv_buf_len)
{
	int rc;
	const struct cs5490_config *cfg = dev->config;
	struct cs5490_data *data = dev->data;

	if ((send_data == NULL) || (len == 0) ||
	     (recv_buf == NULL) || (recv_buf_len == 0)) {
		return -EINVAL;
	}

	rc = uart_rx_enable(cfg->dev, recv_buf, recv_buf_len, 1000 * USEC_PER_MSEC);
	if (rc < 0) {
		return rc;
	}

	printk("transecive data len %u\n", len);
	rc = uart_tx(cfg->dev, send_data, len, 1000 * USEC_PER_MSEC);
	if (rc < 0) {
		return rc;
	}

	if (k_sem_take(&data->uart_rx_sem, K_MSEC(2000))) {
		return -ETIMEDOUT;
	}

	k_msleep(20);

	return 0;
}

static inline int cs5490_select_page(const struct device *dev, uint8_t page_no)
{
	uint8_t select_page = 0;

	select_page = PAGE_SELECT_CMD | page_no;
	return cs5490_send(dev, &select_page, sizeof(select_page));
}

static inline int cs5490_read(const struct device *dev, uint8_t addr,
			      uint8_t *buf, size_t len)
{
	uint8_t read_cmd = 0;

	read_cmd = READ_CMD | addr;
	return cs5490_transceive(dev, &read_cmd, sizeof(read_cmd), buf, len);
}

static inline int cs5490_write(const struct device *dev,
			       uint8_t addr, uint32_t write_data)
{
	uint8_t write_cmd[4] = {0};

	write_cmd[0] = WRITE_CMD | addr;
	sys_put_be24(write_data, &write_cmd[1]);

	return cs5490_send(dev, write_cmd, sizeof(write_cmd));
}

int cs5490_trigger_init(const struct device *dev);
int cs5490_configure_trigger(const struct device *dev,
			     const struct sensor_trigger *trig,
			     sensor_trigger_handler_t handler);

#endif
