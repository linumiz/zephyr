/* W5500 Stand-alone Ethernet Controller with SPI
 *
 * Copyright (c) 2020 Linumiz
 * Author: Parthiban Nallathambi <parthiban@linumiz.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT	wiznet_w5500

#define LOG_MODULE_NAME	eth_w5500
#define LOG_LEVEL	CONFIG_ETHERNET_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr.h>
#include <device.h>
#include <string.h>
#include <errno.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <net/ethernet.h>
#include <ethernet/eth_stats.h>

#include "eth.h"
#include "eth_w5500_priv.h"

#define WIZNET_OUI_B0	0x00
#define WIZNET_OUI_B1	0x08
#define WIZNET_OUI_B2	0xdc

#define W5500_SPI_BLOCK_SELECT(addr)	(((addr) >> 16) & 0x1f)
#define W5500_SPI_READ_CONTROL(addr)	(W5500_SPI_BLOCK_SELECT(addr) << 3)
#define W5500_SPI_WRITE_CONTROL(addr)   \
	((W5500_SPI_BLOCK_SELECT(addr) << 3) | BIT(2))

static int w5500_spi_read(struct device *dev, uint32_t addr,
			  uint8_t *data, uint32_t len)
{
	int ret;
	struct w5500_runtime *ctx = dev->data;
	uint8_t cmd[3] = {
		addr >> 8,
		addr,
		W5500_SPI_READ_CONTROL(addr)
	};
	const struct spi_buf tx_buf = {
		.buf = cmd,
		.len = ARRAY_SIZE(cmd),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf = {
		.buf = data,
		.len = len,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	k_sem_take(&ctx->tx_rx_sem, K_FOREVER);
	ret = spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx);
	k_sem_give(&ctx->tx_rx_sem);

	return ret;
}

static int w5500_spi_write(struct device *dev, uint32_t addr,
			   uint8_t *data, uint32_t len)
{
	int ret;
	struct w5500_runtime *ctx = dev->data;
	uint8_t cmd[3] = {
		addr >> 8,
		addr,
		W5500_SPI_WRITE_CONTROL(addr),
	};
	const struct spi_buf tx_buf[2] = {
		{
			.buf = cmd,
			.len = ARRAY_SIZE(cmd),
		},
		{
			.buf = data,
			.len = len,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	k_sem_take(&ctx->tx_rx_sem, K_FOREVER);
	ret = spi_write(ctx->spi, &ctx->spi_cfg, &tx);
	k_sem_give(&ctx->tx_rx_sem);

	return ret;
}

static int w5500_spi_read8(struct device *dev, uint32_t addr)
{
	int ret;
	struct w5500_runtime *ctx = dev->data;
	uint8_t data[1] = {0};
	uint8_t cmd[3] = {
		addr >> 8,
		addr,
		W5500_SPI_READ_CONTROL(addr)
	};
	const struct spi_buf tx_buf = {
		.buf = cmd,
		.len = ARRAY_SIZE(cmd),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf = {
		.buf = data,
		.len = 1,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	k_sem_take(&ctx->tx_rx_sem, K_FOREVER);
	ret = spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx);
	k_sem_give(&ctx->tx_rx_sem);

	LOG_ERR("%s %d: ret: %d", __func__, __LINE__, ret);
	return data[0];
}

static int w5500_spi_write8(struct device *dev, uint32_t addr,
			   uint8_t data, uint32_t len)
{
	int ret;
	struct w5500_runtime *ctx = dev->data;
	uint8_t cmd[4] = {
		addr >> 8,
		addr,
		W5500_SPI_WRITE_CONTROL(addr),
		data,
	};
	const struct spi_buf tx_buf = {
		.buf = cmd,
		.len = ARRAY_SIZE(cmd),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	LOG_ERR("%s %d %d %d %d", __func__, __LINE__, data, cmd[3], ARRAY_SIZE(cmd));
	k_sem_take(&ctx->tx_rx_sem, K_FOREVER);
	ret = spi_write(ctx->spi, &ctx->spi_cfg, &tx);
	k_sem_give(&ctx->tx_rx_sem);

	return ret;
}

static int w5500_readbuf(struct device *dev, uint16_t offset, uint8_t *buf,
			 int len)
{
	uint32_t addr;
	int remain = 0;
	int ret;
	const uint32_t mem_start = W5500_Sn_RX_MEM_START;
	const uint16_t mem_size = W5500_RX_MEM_SIZE;

	offset %= mem_size;
	addr = mem_start + offset;

	if (offset + len > mem_size) {
		remain = (offset + len) % mem_size;
		len = mem_size - offset;
	}

	ret = w5500_spi_read(dev, addr, buf, len);
	if (ret || !remain) {
		return ret;
	}

	return w5500_spi_read(dev, mem_start, buf + len, remain);
}

static int w5500_writebuf(struct device *dev, uint16_t offset, uint8_t *buf,
			  int len)
{
	uint32_t addr;
	int ret;
	int remain = 0;
	const uint32_t mem_start = W5500_Sn_TX_MEM_START;
	const uint32_t mem_size = W5500_TX_MEM_SIZE;

	offset %= mem_size;
	addr = mem_start + offset;

	if (offset + len > mem_size) {
		remain = (offset + len) % mem_size;
		len = mem_size - offset;
	}


	ret = w5500_spi_write(dev, addr, buf, len);
	if (ret || !remain) {
		return ret;
	}

	return w5500_spi_write(dev, mem_start, buf + len, remain);
}

static void w5500_command(struct device *dev, uint8_t cmd)
{
	uint8_t reg;
	struct w5500_runtime *ctx = dev->data;

	w5500_spi_write(dev, W5500_S0_CR(ctx), &cmd, 1);
	do {
		w5500_spi_read(dev, W5500_S0_CR(ctx), &reg, 1);
		/* TODO break after 100ms - how to do time_after */
		k_msleep(1);
	} while (!reg);
}

static int w5500_tx(struct device *dev, struct net_pkt *pkt)
{
	struct w5500_runtime *ctx = dev->data;
	uint16_t len = net_pkt_get_len(pkt);
	uint16_t offset;
	uint8_t off[2];
	int ret;
	uint8_t buf[NET_ETH_MAX_FRAME_SIZE];

	w5500_spi_read(dev, W5500_S0_TX_WR(ctx), off, 2);
	offset = sys_get_be16(off);

	if (net_pkt_read(pkt, buf, len)) {
		return -EIO;
	}

	ret = w5500_writebuf(dev, offset, buf, len);
	if (ret < 0) {
		return ret;
	}

	sys_put_be16(offset + len, off);
	ret = w5500_spi_write(dev, W5500_S0_TX_WR(ctx), off, 2);
	w5500_command(dev, S0_CR_SEND);

	return 0;
}

static void w5500_isr(struct device *dev)
{
	uint8_t ir;
	uint8_t mask = 0;
	uint8_t header[2];
	uint8_t tmp[2];
	uint16_t rx_buf_len;
	uint16_t rx_len;
	uint16_t off;
	struct w5500_runtime *ctx = dev->data;
	const struct w5500_config *config = dev->config;
	struct net_buf *pkt_buf = NULL;
	struct net_pkt *pkt;

	while (true) {
		k_sem_take(&ctx->int_sem, K_FOREVER);

		w5500_spi_read(dev, W5500_S0_IR(ctx), &ir, 1);
		if (!ir) {
			goto done;
		}

		w5500_spi_write(dev, W5500_S0_IR(ctx), &ir, 1);

		if (ir & S0_IR_SENDOK) {
			LOG_DBG("TX Done");
		}

		if (ir & S0_IR_RECV) {
			/* disable interrupt */
			w5500_spi_write(dev, W5500_SIMR, &mask, 1);
			w5500_spi_read(dev, W5500_S0_RX_RSR(ctx), tmp, 2);
			rx_buf_len = sys_get_be16(tmp);

			if (rx_buf_len == 0) {
				goto done;
			}

			w5500_spi_read(dev, W5500_S0_RX_RD(ctx), tmp, 2);
			off = sys_get_be16(tmp);
			w5500_readbuf(dev, off, header, 2);

			rx_len = sys_get_be16(header) - 2;
			pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, rx_len,
					AF_UNSPEC, 0,
					K_MSEC(config->timeout));
			if (!pkt) {
				LOG_ERR("Could not allocate rx buffer");
				eth_stats_update_errors_rx(ctx->iface);
				goto done;
			}

			pkt_buf = pkt->buffer;

			do {
				size_t frag_len;
				uint8_t *data_ptr;
				int to_read;

				data_ptr = pkt_buf->data;
				frag_len = net_buf_tailroom(pkt_buf);

				to_read = rx_len > frag_len ? frag_len : rx_len;
				w5500_readbuf(dev, off + 2, data_ptr, to_read);
				sys_put_be16(off + 2 + to_read, tmp);
				w5500_spi_write(dev, W5500_S0_RX_RD(ctx), tmp, 2);
				w5500_command(dev, S0_CR_RECV);
				net_buf_add(pkt_buf, to_read);

				rx_len -= to_read;
				pkt_buf = pkt_buf->frags;
			} while (rx_len > 0);

			if (net_recv_data(net_pkt_iface(pkt), pkt) < 0) {
				net_pkt_unref(pkt);
			}
		}
	}

done:
	/* enable interrupt */
	mask = IR_S0;
	w5500_spi_write(dev, W5500_SIMR, &mask, 1);
}

static void w5500_iface_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct w5500_runtime *ctx = dev->data;

	LOG_ERR("%s %d", __func__, __LINE__);
	net_if_set_link_addr(iface, ctx->mac_addr,
			     sizeof(ctx->mac_addr),
			     NET_LINK_ETHERNET);
	ethernet_init(iface);

	if (!ctx->iface) {
		ctx->iface = iface;
	}
}

static enum ethernet_hw_caps w5500_get_capabilities(struct device *dev)
{
	ARG_UNUSED(dev);

	LOG_ERR("%s %d", __func__, __LINE__);
	return ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	| ETHERNET_PROMISC_MODE
#endif
	;
}

static int w5500_set_config(struct device *dev, enum ethernet_config_type type,
			    const struct ethernet_config *config)
{
	uint8_t mode;
	uint8_t mr = W5500_S0_MR_MF;
	struct w5500_runtime *ctx = dev->data;

	LOG_ERR("%s %d", __func__, __LINE__);
	w5500_spi_read(dev, W5500_S0_MR(ctx), &mode, 1);
	if (IS_ENABLED(CONFIG_NET_PROMISCUOUS_MODE) &&
	    type == ETHERNET_CONFIG_TYPE_PROMISC_MODE) {
		if (config->promisc_mode) {
			if (!(mode & W5500_S0_MR_MF))
				return -EALREADY;
			}

			/* clear */
			WRITE_BIT(mode, mr, 0);
	} else {
		if (mode & mr) {
			return -EALREADY;
		}

		/* set */
		WRITE_BIT(mode, mr, 1);
	}

	return w5500_spi_write(dev, W5500_S0_MR(ctx), &mode, 1);
}

static int w5500_hw_start(struct device *dev)
{
	uint8_t mode = S0_MR_MACRAW;
	uint8_t mask = IR_S0;
	struct w5500_runtime *ctx = dev->data;

	LOG_ERR("%s %d", __func__, __LINE__);
	w5500_spi_write(dev, W5500_S0_MR(ctx), &mode, 1);
	w5500_command(dev, S0_CR_OPEN);
	/* enable interrupt */
	w5500_spi_write(dev, W5500_SIMR, &mask, 1);

	return 0;
}

static int w5500_hw_stop(struct device *dev)
{
	uint8_t mask = 0;

	LOG_ERR("%s %d", __func__, __LINE__);
	/* disable interrupt */
	w5500_spi_write(dev, W5500_SIMR, &mask, 1);
	w5500_command(dev, S0_CR_CLOSE);

	return 0;
}

static struct ethernet_api w5500_api_funcs = {
	.iface_api.init = w5500_iface_init,
	.get_capabilities = w5500_get_capabilities,
	.set_config = w5500_set_config,
	.start = w5500_hw_start,
	.stop = w5500_hw_stop,
	.send = w5500_tx,
};

static int w5500_hw_reset(struct device *dev)
{
	uint8_t mask = 0;
	uint8_t tmp = MR_RST;

	w5500_spi_write(dev, W5500_MR, &tmp, 1);
	tmp = MR_PB;
	w5500_spi_write(dev, W5500_MR, &tmp, 1);

	/* disable interrupt */
	return w5500_spi_write(dev, W5500_SIMR, &mask, 1);
}

static void w5500_gpio_callback(struct device *dev,
				struct gpio_callback *cb,
				uint32_t pins)
{
	struct w5500_runtime *ctx =
		CONTAINER_OF(cb, struct w5500_runtime, gpio_cb);

	k_sem_give(&ctx->int_sem);
}

static void w5500_set_macaddr(struct device *dev)
{
	struct w5500_runtime *ctx = dev->data;

	/* override vendor bytes */
	ctx->mac_addr[0] = WIZNET_OUI_B0;
	ctx->mac_addr[1] = WIZNET_OUI_B1;
	ctx->mac_addr[2] = WIZNET_OUI_B2;
	if (ctx->generate_mac) {
		ctx->generate_mac(ctx->mac_addr);
	}

	w5500_spi_write(dev, W5500_SHAR, ctx->mac_addr, sizeof(ctx->mac_addr));
}

static void w5500_memory_configure(struct device *dev)
{
	int i;
	uint8_t mem = 0x10;

	/* Configure RX & TX memory to 16K */
	w5500_spi_write(dev, W5500_Sn_RXMEM_SIZE(0), &mem, 1);
	w5500_spi_write(dev, W5500_Sn_TXMEM_SIZE(0), &mem, 1);

	mem = 0;
	for (i = 1; i < 8; i++) {
		w5500_spi_write(dev, W5500_Sn_RXMEM_SIZE(i), &mem, 1);
		w5500_spi_write(dev, W5500_Sn_TXMEM_SIZE(i), &mem, 1);
	}
}

static void w5500_random_mac(uint8_t *mac_addr)
{
	gen_random_mac(mac_addr, WIZNET_OUI_B0, WIZNET_OUI_B1, WIZNET_OUI_B2);
}

static int w5500_init(struct device *dev)
{
	int err;
	uint8_t rtr[2];
	const struct w5500_config *config = dev->config;
	struct w5500_runtime *ctx = dev->data;

	ctx->spi_cfg.operation = SPI_WORD_SET(8);
	ctx->spi_cfg.frequency = config->spi_freq;
	ctx->spi_cfg.slave = config->spi_slave;

	ctx->spi = device_get_binding((char *)config->spi_port);
	if (!ctx->spi) {
		LOG_ERR("SPI master port %s not found", config->spi_port);
		return -EINVAL;
	}

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	ctx->spi_cs.gpio_dev = device_get_binding((char *)config->spi_cs_port);
	if (!ctx->spi_cs.gpio_dev) {
		LOG_ERR("SPI CS port %s not found", config->spi_cs_port);
		return -EINVAL;
	}

	ctx->spi_cs.gpio_pin = config->spi_cs_pin;
	ctx->spi_cs.gpio_dt_flags = config->spi_cs_dt_flags;
	ctx->spi_cfg.cs = &ctx->spi_cs;
#endif
	ctx->gpio = device_get_binding((char *)config->gpio_port);
	if (!ctx->gpio) {
		LOG_ERR("GPIO port %s not found", config->gpio_port);
		return -EINVAL;
	}

	if (gpio_pin_configure(ctx->gpio, config->gpio_pin,
			       GPIO_INPUT | config->gpio_flags)) {
		LOG_ERR("Unable to configure GPIO pin %u", config->gpio_pin);
		return -EINVAL;
	}

	gpio_init_callback(&(ctx->gpio_cb), w5500_gpio_callback,
			   BIT(config->gpio_pin));

	if (gpio_add_callback(ctx->gpio, &(ctx->gpio_cb))) {
		return -EINVAL;
	}

	gpio_pin_interrupt_configure(ctx->gpio,
				     config->gpio_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);

	ctx->reset = device_get_binding((char *)config->reset_port);
	if (!ctx->reset) {
		LOG_ERR("GPIO port %s not found", config->reset_port);
		return -EINVAL;
	}

	if (gpio_pin_configure(ctx->reset, config->reset_pin,
			       GPIO_OUTPUT | config->reset_flags)) {
		LOG_ERR("Unable to configure GPIO pin %u", config->reset_pin);
		return -EINVAL;
	}

#define GPIO_RESET_PIN          DT_INST_GPIO_PIN(0, reset_gpios)
	gpio_pin_set(ctx->reset, GPIO_RESET_PIN, 0);
	k_msleep(1);
	gpio_pin_set(ctx->reset, GPIO_RESET_PIN, 1);
	k_msleep(5);

	err = w5500_hw_reset(dev);
	if (err) {
		LOG_ERR("Reset failed");
		return err;
	}

	w5500_set_macaddr(dev);
	w5500_memory_configure(dev);

	uint8_t ip = 0xc0;
	uint8_t read = 0;
	err = w5500_spi_write8(dev, W5500_GW, ip, 1);
	read = w5500_spi_read8(dev, W5500_GW);

	LOG_ERR("%s %d IP: %d %x ret: %d", __func__, __LINE__, read, read, err);
	/* check retry time value */
/*
	w5500_spi_read(dev, W5500_RTR, rtr, 2);
	LOG_ERR("RTR: %d-%d: %d\n", rtr[0], rtr[1], sys_get_be16(rtr));
	if (sys_get_be16(rtr) != RTR_DEFAULT) {
		LOG_ERR("Unable to read RTR register");
		return -ENODEV;
	}
*/
	k_thread_create(&ctx->thread, ctx->thread_stack,
			CONFIG_ETH_W5500_RX_THREAD_STACK_SIZE,
			(k_thread_entry_t)w5500_isr,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ETH_W5500_RX_THREAD_PRIO),
			0, K_NO_WAIT);

	LOG_INF("W5500 Initialized");

	return 0;
}

static struct w5500_runtime w5500_0_runtime = {
#if 0
#if NODE_HAS_VALID_MAC_ADDR(DT_DRV_INST(0))
	.mac_addr = DT_INST_PROP(0, local_mac_address),
	.generate_mac = NULL,
#else
	.generate_mac = w5500_random_mac,
#endif
#endif
	.generate_mac = w5500_random_mac,
	.tx_rx_sem = Z_SEM_INITIALIZER(w5500_0_runtime.tx_rx_sem,
				       1,  UINT_MAX),
	.int_sem  = Z_SEM_INITIALIZER(w5500_0_runtime.int_sem,
				      0, UINT_MAX),
};

static const struct w5500_config w5500_0_config = {
	.gpio_port = DT_INST_GPIO_LABEL(0, int_gpios),
	.gpio_pin = DT_INST_GPIO_PIN(0, int_gpios),
	.gpio_flags = DT_INST_GPIO_FLAGS(0, int_gpios),
	.reset_port = DT_INST_GPIO_LABEL(0, reset_gpios),
	.reset_pin = DT_INST_GPIO_PIN(0, reset_gpios),
	.reset_flags = DT_INST_GPIO_FLAGS(0, reset_gpios),
	.spi_port = DT_INST_BUS_LABEL(0),
	.spi_freq  = DT_INST_PROP(0, spi_max_frequency),
	.spi_slave = DT_INST_REG_ADDR(0),
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	.spi_cs_port = DT_INST_SPI_DEV_CS_GPIOS_LABEL(0),
	.spi_cs_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0),
	.spi_cs_dt_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0),
#endif
	.timeout = CONFIG_ETH_W5500_TIMEOUT,
};

ETH_NET_DEVICE_INIT(eth_w5500, DT_INST_LABEL(0),
		    w5500_init, device_pm_control_nop,
		    &w5500_0_runtime, &w5500_0_config,
		    CONFIG_ETH_INIT_PRIORITY, &w5500_api_funcs, NET_ETH_MTU);
