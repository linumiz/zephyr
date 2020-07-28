static const struct socket_op_vtable w5500_socket_fd_op_vtable;

#define W5500_SPI_BLOCK_SELECT(addr)	(((addr) >> 16) & 0x1f)
#define W5500_SPI_READ_CONTROL(addr)	(W5500_SPI_BLOCK_SELECT(addr) << 3)
#define W5500_SPI_WRITE_CONTROL(addr)   \
	((W5500_SPI_BLOCK_SELECT(addr) << 3) | BIT(2))

static int w5500_spi_read(struct device *dev, uint32_t addr, uint8_t *data)
{
	struct w5500_runtime *ctx = dev->driver_data;
	uint8_t cmd[3] = {
		addr >> 8,
		addr,
		W5500_SPI_READ_CONTROL(addr)
	};
	const struct spi_buf tx_buf = {
		.buf = cmd,
		.len = 4,
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

	return spi_transceive(context->spi, &context->spi_cfg, &tx, &rx);
}

static int w5500_spi_write(struct device *dev, uint32_t addr, uint8_t data)
{
	struct w5500_runtime *ctx = dev->driver_data;
	uint8_t cmd[4] = {
		addr >> 8,
		addr,
		W5500_SPI_WRITE_CONTROL(addr),
		data
	};
	const struct spi_buf tx_buf = {
		.buf = cmd,
		.len = 4,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	return spi_write(context->spi, &context->spi_cfg, &tx);
}

static int w5500_socket_bind(void *obj, const struct sockaddr *addr,
			     socklen_t addrlen)
{
}

static int w5500_socket_connect(void *obj, const struct sockaddr *addr,
				socklen_t addrlen)
{
}

static int w5500_socket_accept(void *obj, const struct sockaddr *addr,
			       socklen_t addrlen)
{
}

static int w5500_socket_listen(void *obj, int backlog)
{
}

static ssize_t w5500_socket_sendmsg(void *obj, const struct msghdr *msg,
				    int flags)
{
}

static ssize_t w5500_socket_sendto(void *obj, const void *buf, size_t len,
				   int flags, const struct sockaddr *dest_addr,
				   socklen_t addrlen)
{
}

static ssize_t w5500_socket_recvfrom(void *obj, void *buf, size_t max_len,
				     int flags, struct sockaddr *src_addr,
				     socklen_t *addrlen)
{
}

static int w5500_socket_setsockopt(void *obj, int level, int optname,
				   const void *optval, socklen_t optlen)
{
}

static ssize_t w5500_socket_read(void *obj, void *buffer, size_t count)
{
	return w5500_socket_recvfrom(obj, buffer, count, 0, NULL, 0);
}

static ssize_t w5500_socket_write(void *obj, const void *buffer,
				   size_t count)
{
	return w5500_socket_sendto(obj, buffer, count, 0, NULL, 0);
}

static int w5500_socket_ioctl(void *obj, unsigned int request, va_list args)
{
}

static int w5500_socket_open(int family, int type, int proto)
{
}

static int w5500_socket_create(int family, int type, int proto)
{
	int fd = z_reserve_fd();
	int sock;

	if (fd < 0) {
		return -1;
	}

	sock = w5500_socket_open(family, type, proto);
	if (sock < 0) {
		z_free_fd(fd);
		return -1;
	}

	z_finalize_fd(fd, SD_TO_OBJ(sock),
		      (const struct fd_op_vtable *)
					&w5500_socket_fd_op_vtable);

	return fd;
}

static const struct socket_op_vtable w5500_socket_fd_op_vtable = {
	.fd_vtable = {
		.read = w5500_socket_read,
		.write = w5500_socket_write,
		.ioctl = w5500_socket_ioctl,
	},
	.bind = w5500_socket_bind,
	.connect = w5500_socket_connect,
	.accept = w5500_socket_accept,
	.listen = w5500_socket_listen,
	.sendmsg = w5500_socket_sendmsg,
	.sendto = w5500_socket_sendto,
	.recvfrom = w5500_socket_recvfrom,
	.setsockopt = w5500_socket_setsockopt,
	/* TODO TLS issue: 27318 */
	.getsockopt = NULL,
	.setsockopt = NULL,
};

NET_SOCKET_REGISTER(w5500, AF_UNSPEC, w5500_socket_is_supported,
		    w5500_socket_create);

static w5500_net_iface_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct w5500_runtime *ctx = dev->driver_data;

	iface->if_dev->offload = &w5500_net_offload;
	net_if_set_link_addr(iface, w5500_get_mac(dev),
			     sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);
	data->net_iface = iface;
	/* FIXME DNS */
}

static struct net_if_api w5500_api_funcs = {
	.init = w5500_net_iface_init,
};

static int w5500_hw_reset(struct device *dev)
{
	w5500_spi_write(dev, W5500_MR, MR_RST);
	k_msleep(5);
	w5500_spi_write(dev, W5500_MR, MR_PB);

	w5500_disable_intr(dev, W5500_SIMR, 0);
	w5500_init_macaddr(dev);
	w5500_memory_configure(dev);
}

static int w5500_init(struct device *dev)
{
	int err;
	const struct w5500_config *config = dev->config_info;
	struct w5500_runtime *ctx = dev->driver_data;

	ctx->spi_cfg.frequency = config->spi_freq;
	ctx->spi_cfg.slave = config->spi_slave;

	ctx->spi = device_get_binding((char *)config->spi_port);
	if (!ctx->spi) {
		LOG_ERR("SPI master port %s not found", config->spi_port);
		return -EINVAL;
	}

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	ctx->spi_cs.gpio_dev = device_get_binding((char *)config->spi_cs_port);
	if (ctx->spi_cs.gpio_dev) {
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

	gpio_init_callback(&(ctx->gpio_cb), eth_enc28j60_gpio_callback,
			   BIT(config->gpio_pin));

	if (gpio_add_callback(ctx->gpio, &(ctx->gpio_cb))) {
		return -EINVAL;
	}

	gpio_pin_interrupt_configure(ctx->gpio,
				     config->gpio_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);

	err = w5500_hw_reset(dev);
	if (err) {
		return err;
	}
}

static struct w5500_runtime w5500_0_runtime = {
	.tx_rx_sem = Z_SEM_INITIALIZER(w5500_0_runtime.tx_rx_sem,
				       1,  UINT_MAX),
	.int_sem  = Z_SEM_INITIALIZER(w5500_0_runtime.int_sem,
				      0, UINT_MAX),
};

static const struct w5500_config w5500_0_config = {
	.gpio_port = DT_INST_GPIO_LABEL(0, int_gpios),
	.gpio_pin = DT_INST_GPIO_PIN(0, int_gpios),
	.gpio_flags = DT_INST_GPIO_FLAGS(0, int_gpios),
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

NET_DEVICE_OFFLOAD_INIT(w5500_0, DT_INST_LABEL(0),
		    w5500_init, device_pm_control_nop,
		    &w5500_0_runtime, &w5500_0_config,
		    CONFIG_ETH_INIT_PRIORITY, &w5500_api_funcs, NET_ETH_MTU);
