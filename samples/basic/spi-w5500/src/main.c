/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/spi.h>

#define SPI_DRV_NAME		"SPI_0"
#define CS_CONTROL_GPIO_PIN	"GPIO_3"

#define GPIO_CS_PIN 		0	

#define _W5500_SPI_VDM_OP_	0x00
#define _W5500_SPI_FDM_OP_LEN1_	0x01
#define _W5500_SPI_FDM_OP_LEN2_ 0x02
#define _W5500_SPI_FDM_OP_LEN4_	0x03

#define _W5500_SPI_READ_	(0x00 << 2)

#define _W5500_IO_BASE_		0x00000000
#define WIZCHIP_CREG_BLOCK	0x00
#define VERSIONR		(_W5500_IO_BASE_ + (0x0039 << 8) + (WIZCHIP_CREG_BLOCK << 3))

struct device *spi_dev;
struct device *cs_gpio;
struct spi_config spi_cfg_fast;

static int w5500_read(uint32_t addr, uint8_t *buf, int len)
{
	uint8_t buffer_tx[3];

	buffer_tx[0] = 0x00;
	buffer_tx[1] = 0x39;
	buffer_tx[2] = 0x01;

	uint8_t buffer_rx[4] = {0};
	int ret = 0;

	printk("Received data: %x %x %x\n", buffer_tx[0], buffer_tx[1], buffer_tx[2]);
	printk("Received data: %x %x %x %x\n", buffer_rx[0], buffer_rx[1], buffer_rx[2], buffer_rx[3]);
	const struct spi_buf tx_buf = {
		.buf = buffer_tx,
		.len = 3,
	};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	const struct spi_buf rx_buf = {
		.buf = buffer_rx,
		.len = 4,
	};

	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	ret = spi_transceive(spi_dev, &spi_cfg_fast, &tx, &rx);
	printk("ret: %d\n", ret);
	if (ret < 0) {
		printk("Failed to transceive data\n");
	}

	printk("Received data: %x %x %x %x\n", buffer_rx[0], buffer_rx[1], buffer_rx[2], buffer_rx[3]);
}

static int w5500_read(uint8_t *buf, int len)
{
}

static int w5500_write(uint8_t *buf, int len)
{
}

void main(void)
{
	struct spi_cs_control spi_cs;

	spi_cs.gpio_dev = cs_gpio,
	spi_cs.gpio_pin = GPIO_CS_PIN,
	spi_cs.gpio_dt_flags = GPIO_ACTIVE_LOW,
	spi_cs.delay = 0,

	spi_cfg_fast.frequency = 24000000;
	spi_cfg_fast.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	spi_cfg_fast.cs = &spi_cs;

	spi_dev = device_get_binding(SPI_DRV_NAME);
	if (!spi_dev) {
		printk("Cannot find %s!\n", SPI_DRV_NAME);
		return;
	}

/*
	cs_gpio = device_get_binding(CS_CONTROL_GPIO_PIN);
	if (!cs_gpio) {
		printk("Cannot get pointer to %s device\n",
				CS_CONTROL_GPIO_PIN);
		return;
	}
*/
	w5500_get_version();
}
