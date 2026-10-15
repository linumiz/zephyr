/*
 * Copyright (c) 2026 Bang & Olufsen A/S, Denmark
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_spi

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mspm0_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(spi_mspm0, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

/* SPI registers */
#define MSPM0_SPI_PWREN  0x800
#define MSPM0_SPI_CLKDIV 0x1000
#define MSPM0_SPI_CLKSEL 0x1004
#define MSPM0_SPI_IIDX   0x1020
#define MSPM0_SPI_IMASK  0x1028
#define MSPM0_SPI_ICLR   0x1048
#define MSPM0_SPI_CTL0   0x1100
#define MSPM0_SPI_CTL1   0x1104
#define MSPM0_SPI_CLKCTL 0x1108
#define MSPM0_SPI_IFLS   0x110C
#define MSPM0_SPI_STAT   0x1110
#define MSPM0_SPI_RXDATA 0x1130
#define MSPM0_SPI_TXDATA 0x1140

/* Power enable*/
#define MSPM0_SPI_PWREN_KEY         (0x26U << 24)
#define MSPM0_SPI_PWREN_ENABLE_MASK BIT(0)

/* Control register-0 */
#define MSPM0_SPI_CTL0_DSS_MASK   GENMASK(4, 0)
#define MSPM0_SPI_CTL0_FRF_MASK   GENMASK(6, 5)
#define MSPM0_SPI_CTL0_SPO_MASK   BIT(8)
#define MSPM0_SPI_CTL0_SPH_MASK   BIT(9)
#define MSPM0_SPI_CTL0_CSSEL_MASK GENMASK(13, 12)
#define MSPM0_SPI_CTL0_CSCLR_MASK BIT(14)

/* Frame format selection */
#define MSPM0_SPI_CTL0_FRF_MOTOROLA_4WIRE FIELD_PREP(MSPM0_SPI_CTL0_FRF_MASK, 1)
#define MSPM0_SPI_CTL0_FRF_TI             FIELD_PREP(MSPM0_SPI_CTL0_FRF_MASK, 2)

/* Control register-1 */
#define MSPM0_SPI_CTL1_ENABLE_MASK    BIT(0)
#define MSPM0_SPI_CTL1_LBM_MASK       BIT(1)
#define MSPM0_SPI_CTL1_CP_MASK        BIT(2)
#define MSPM0_SPI_CTL1_POD_MASK       BIT(3)
#define MSPM0_SPI_CTL1_MSB_MASK       BIT(4)
#define MSPM0_SPI_CTL1_RXTIMEOUT_MASK GENMASK(29, 24)

/* Clock control */
#define MSPM0_SPI_CLKCTL_SCR_MASK   GENMASK(9, 0)
#define MSPM0_SPI_CLKDIV_RATIO_MASK GENMASK(2, 0)
#define MSPM0_SPI_SCR_MIN           0
#define MSPM0_SPI_SCR_MAX           1023

/* Interrupt FIFO Level */
#define MSPM0_SPI_IFLS_RXIFLSEL_MASK GENMASK(5, 3)

/* Status register*/
#define MSPM0_SPI_STAT_TFE_MASK  BIT(0) /* TX FIFO empty */
#define MSPM0_SPI_STAT_TNF_MASK  BIT(1) /* TX FIFO not full */
#define MSPM0_SPI_STAT_RFE_MASK  BIT(2) /* RX FIFO empty */
#define MSPM0_SPI_STAT_RNF_MASK  BIT(3) /* RX FIFO not full */
#define MSPM0_SPI_STAT_BUSY_MASK BIT(4)

/* CPU_INT bit positions */
#define MSPM0_SPI_INT_RXFIFO_OVF_MASK      BIT(0)
#define MSPM0_SPI_INT_RTOUT_MASK           BIT(2)
#define MSPM0_SPI_INT_RX_MASK              BIT(3)
#define MSPM0_SPI_INT_END_OF_TRANSMIT_MASK BIT(6)
#define MSPM0_SPI_INT_RXFULL_MASK          BIT(10)

/* IIDX - interrupt index values */
#define MSPM0_SPI_IIDX_STAT_NONE            0x0U
#define MSPM0_SPI_IIDX_STAT_RXFIFO_OVF      0x1U
#define MSPM0_SPI_IIDX_STAT_RTOUT           0x3U
#define MSPM0_SPI_IIDX_STAT_RX              0x4U
#define MSPM0_SPI_IIDX_STAT_END_OF_TRANSMIT 0x7U
#define MSPM0_SPI_IIDX_STAT_RXFULL          0xBU

#define MSPM0_SPI_MAX_FIFO_DEPTH          4U
#define MSPM0_SPI_IMASK_DISABLE           0U
#define MSPM0_SPI_DFS_8BIT                1U
#define MSPM0_SPI_DFS_16BIT               2U
#define MSPM0_SPI_ICLR_MASK               GENMASK(10, 0)
#define MSPM0_SPI_CTL1_ENABLE             FIELD_PREP(MSPM0_SPI_CTL1_ENABLE_MASK, 1)
#define MSPM0_SPI_CTL1_DISABLE            FIELD_PREP(MSPM0_SPI_CTL1_ENABLE_MASK, 0)
#define SPI_CONTROLLER_DSS_MIN            4
#define SPI_PERIPHERAL_DSS_MIN            7
#define SPI_DSS_MAX                       16
#define MSPM0_SPI_CS_LINE_MIN             0U
#define MSPM0_SPI_CS_LINE_MAX             3U
#define MSPM0_SPI_CONTROLLER_RX_FIFO_TRIG 2U
#define MSPM0_SPI_PERIPHERAL_RX_FIFO_TRIG 7U
#define MSPM0_SPI_RXTIMEOUT_MAX           63U

/* Computes the minimum number of bytes per frame using word_size */
#define BYTES_PER_FRAME(word_size) (((word_size) + 7) / 8)

/* Interrupts enabled in controller mode*/
#define SPI_MSPM0_IMASK_CONTROLLER_ENABLE                                                          \
	(MSPM0_SPI_INT_RX_MASK | MSPM0_SPI_INT_RXFULL_MASK | MSPM0_SPI_INT_END_OF_TRANSMIT_MASK |  \
	 MSPM0_SPI_INT_RXFIFO_OVF_MASK)

#define SPI_MSPM0_IMASK_PERIPHERAL_ENABLE                                                          \
	(MSPM0_SPI_INT_RX_MASK | MSPM0_SPI_INT_RXFULL_MASK | MSPM0_SPI_INT_RXFIFO_OVF_MASK |       \
	 MSPM0_SPI_INT_RTOUT_MASK)

/* Helper macros */
#define SPI_UPDATE_REG(reg_addr, value, mask)                                                      \
	{                                                                                          \
		uint32_t tmp = sys_read32(reg_addr);                                               \
		tmp = (tmp & ~(mask)) | ((value) & (mask));                                        \
		sys_write32(tmp, reg_addr);                                                        \
	}

#define SPI_CONTEXT_CS_CONTROL_IF_CONTROLLER(ctx, state)                                           \
	if (!spi_context_is_peripheral(ctx)) {                                                     \
		spi_context_cs_control(ctx, state);                                                \
	}

struct spi_mspm0_config {
	uint32_t base;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clk_dev;
	struct mspm0_sys_clock clk_subsys;
	void (*irq_config_func)(const struct device *dev);
	bool cs_gpios_present;
};

struct spi_mspm0_data {
	struct spi_context spi_ctx;
	size_t tf_size;     /* size upto which transfer is to be performed */
	uint8_t dfs;        /* Data frame size in bytes */
	size_t frames_left; /* Total frame count */
};

static void spi_mspm0_tx_fill(uint32_t base, struct spi_mspm0_data *data)
{
	struct spi_context *ctx = &data->spi_ctx;
	uint32_t tx_data;
	bool peripheral = spi_context_is_peripheral(ctx);

	while (data->frames_left &&
	       (sys_read32(base + MSPM0_SPI_STAT) & MSPM0_SPI_STAT_TNF_MASK) &&
	       (peripheral || data->tf_size < MSPM0_SPI_MAX_FIFO_DEPTH)) {

		tx_data = 0;
		if (spi_context_tx_buf_on(ctx)) {
			memcpy(&tx_data, ctx->tx_buf, data->dfs);
		}
		sys_write32(tx_data, base + MSPM0_SPI_TXDATA);
		spi_context_update_tx(ctx, data->dfs, 1);
		data->tf_size++;
		data->frames_left--;
	}
}

static void spi_mspm0_rx_drain(uint32_t base, struct spi_mspm0_data *data)
{
	struct spi_context *ctx = &data->spi_ctx;
	uint32_t rx_data;

	while (data->tf_size > 0) {

		if (sys_read32(base + MSPM0_SPI_STAT) & MSPM0_SPI_STAT_RFE_MASK) {
			LOG_DBG("RX EMPTY...");
			break;
		}

		rx_data = sys_read32(base + MSPM0_SPI_RXDATA);
		if (spi_context_rx_buf_on(ctx)) {
			memcpy(ctx->rx_buf, &rx_data, data->dfs);
		}
		spi_context_update_rx(ctx, data->dfs, 1);
		data->tf_size--;
	}
}

static void spi_mspm0_finish(const struct device *dev, int status)
{
	struct spi_mspm0_data *data = dev->data;
	struct spi_context *ctx = &data->spi_ctx;

	SPI_CONTEXT_CS_CONTROL_IF_CONTROLLER(ctx, false);
	spi_context_complete(ctx, dev, status);
}

static void spi_mspm0_buffer_clear(uint32_t base, struct spi_mspm0_data *data)
{
	/* drain leftover RX frames */
	while (!(sys_read32(base + MSPM0_SPI_STAT) & MSPM0_SPI_STAT_RFE_MASK)) {
		(void)sys_read32(base + MSPM0_SPI_RXDATA);
	}
	sys_write32(MSPM0_SPI_ICLR_MASK, base + MSPM0_SPI_ICLR);
	data->tf_size = 0;
}

static int spi_mspm0_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_mspm0_config *cfg = dev->config;
	struct spi_mspm0_data *data = dev->data;
	uint32_t ctl0 = 0;
	uint32_t ctl1 = 0;
	uint32_t clock_rate;
	uint32_t word_size;
	uint32_t dss_min;
	uint32_t ifls;
	uint32_t scr = 0;
	bool peripheral = (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_CONTROLLER);
	bool hw_cs;
	int ret;

	if (spi_context_configured(&data->spi_ctx, config)) {
		return 0;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    (config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		return -EINVAL;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		return -ENOTSUP; /* Does not support half-duplex mode */
	}

	hw_cs = (peripheral || (!spi_cs_is_gpio(config)));

	if (hw_cs) {

		if (!IN_RANGE(config->peripheral, MSPM0_SPI_CS_LINE_MIN, MSPM0_SPI_CS_LINE_MAX)) {
			LOG_ERR("CS line out of range (0..%u)", MSPM0_SPI_CS_LINE_MAX);
			return -EINVAL;
		}

		if (config->operation & (SPI_CS_ACTIVE_HIGH | SPI_HOLD_ON_CS)) {
			LOG_ERR("native CS does not support active-high/hold-on-cs, use cs-gpios");
			return -ENOTSUP;
		}

		ctl0 |= FIELD_PREP(MSPM0_SPI_CTL0_CSSEL_MASK, config->peripheral);
	}

	if (peripheral) {
		ctl1 &= ~(MSPM0_SPI_CTL1_CP_MASK); /*peripheral mode */
		dss_min = SPI_PERIPHERAL_DSS_MIN;
		ctl0 |= MSPM0_SPI_CTL0_CSCLR_MASK;

		if (!IS_ENABLED(CONFIG_SPI_PERIPHERAL)) {
			LOG_ERR("peripheral mode needs CONFIG_SPI_PERIPHERAL");
			return -ENOTSUP;
		}

		/* fifo interupt level set to > 1 frame */
		ifls = FIELD_PREP(MSPM0_SPI_IFLS_RXIFLSEL_MASK, MSPM0_SPI_PERIPHERAL_RX_FIFO_TRIG);
		ctl1 &= ~MSPM0_SPI_CTL1_POD_MASK;
		ctl1 |= FIELD_PREP(MSPM0_SPI_CTL1_RXTIMEOUT_MASK, MSPM0_SPI_RXTIMEOUT_MAX);

	} else {
		ctl1 |= MSPM0_SPI_CTL1_CP_MASK; /*controller mode */
		ret = clock_control_get_rate(cfg->clk_dev, (clock_control_subsys_t)&cfg->clk_subsys,
					     &clock_rate);
		if (ret < 0) {
			return ret;
		}

		if (config->frequency == 0 || config->frequency > clock_rate / 2) {
			return -EINVAL;
		}

		dss_min = SPI_CONTROLLER_DSS_MIN;
		scr = DIV_ROUND_UP(clock_rate, 2 * config->frequency) - 1;

		if (!IN_RANGE(scr, MSPM0_SPI_SCR_MIN, MSPM0_SPI_SCR_MAX)) {
			return -EINVAL;
		}
		/* fifo interupt level set to half the FIFO capacity */
		ifls = FIELD_PREP(MSPM0_SPI_IFLS_RXIFLSEL_MASK, MSPM0_SPI_CONTROLLER_RX_FIFO_TRIG);
	}

	if (config->operation & SPI_FRAME_FORMAT_TI) {
		ctl0 |= MSPM0_SPI_CTL0_FRF_TI;
	} else { /* CPOL/CPHA only supported in Motorola frame format */
		ctl0 |= MSPM0_SPI_CTL0_FRF_MOTOROLA_4WIRE;
		ctl0 |= (config->operation & SPI_MODE_CPOL) ? MSPM0_SPI_CTL0_SPO_MASK : 0;
		ctl0 |= (config->operation & SPI_MODE_CPHA) ? MSPM0_SPI_CTL0_SPH_MASK : 0;
	}

	word_size = SPI_WORD_SIZE_GET(config->operation);

	if (!IN_RANGE(word_size, dss_min, SPI_DSS_MAX)) {
		return -ENOTSUP;
	}

	/* hand the CS to whoever drives CS for this configuration */
	if (hw_cs) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	} else {
		ret = gpio_pin_configure_dt(&config->cs.gpio, GPIO_OUTPUT_INACTIVE);
	}

	if (ret < 0) {
		return ret;
	}

	data->dfs = BYTES_PER_FRAME(word_size);

	ctl0 |= FIELD_PREP(MSPM0_SPI_CTL0_DSS_MASK, (word_size - 1));
	ctl1 |= (config->operation & SPI_TRANSFER_LSB) ? 0 : MSPM0_SPI_CTL1_MSB_MASK;
	ctl1 |= (config->operation & SPI_MODE_LOOP) ? MSPM0_SPI_CTL1_LBM_MASK : 0;

	SPI_UPDATE_REG(cfg->base + MSPM0_SPI_CTL1, MSPM0_SPI_CTL1_DISABLE,
		       MSPM0_SPI_CTL1_ENABLE_MASK)

	SPI_UPDATE_REG(cfg->base + MSPM0_SPI_CTL0, ctl0,
		       MSPM0_SPI_CTL0_DSS_MASK | MSPM0_SPI_CTL0_FRF_MASK |
		       MSPM0_SPI_CTL0_SPO_MASK | MSPM0_SPI_CTL0_SPH_MASK |
		       MSPM0_SPI_CTL0_CSSEL_MASK | MSPM0_SPI_CTL0_CSCLR_MASK)

	SPI_UPDATE_REG(cfg->base + MSPM0_SPI_CTL1, ctl1,
		       MSPM0_SPI_CTL1_RXTIMEOUT_MASK | MSPM0_SPI_CTL1_LBM_MASK |
			       MSPM0_SPI_CTL1_CP_MASK | MSPM0_SPI_CTL1_MSB_MASK)

	SPI_UPDATE_REG(cfg->base + MSPM0_SPI_IFLS, ifls, MSPM0_SPI_IFLS_RXIFLSEL_MASK)

	if (peripheral) {
		SPI_UPDATE_REG(cfg->base + MSPM0_SPI_CTL1, ctl1, MSPM0_SPI_CTL1_POD_MASK)
	} else {
		SPI_UPDATE_REG(cfg->base + MSPM0_SPI_CLKCTL,
			       FIELD_PREP(MSPM0_SPI_CLKCTL_SCR_MASK, scr),
			       MSPM0_SPI_CLKCTL_SCR_MASK)
	}

	SPI_UPDATE_REG(cfg->base + MSPM0_SPI_CTL1, MSPM0_SPI_CTL1_ENABLE,
		       MSPM0_SPI_CTL1_ENABLE_MASK)
	data->spi_ctx.config = config;

	return 0;
}

static void spi_mspm0_isr(const struct device *dev)
{
	const struct spi_mspm0_config *cfg = dev->config;
	struct spi_mspm0_data *data = dev->data;
	struct spi_context *ctx = &data->spi_ctx;
	uint32_t base = cfg->base;
	int isr_status = 0;
	uint32_t iidx = sys_read32(base + MSPM0_SPI_IIDX);

	if (iidx == MSPM0_SPI_IIDX_STAT_NONE) {
		return;
	}

	switch (iidx) {
	case MSPM0_SPI_IIDX_STAT_RX:
	case MSPM0_SPI_IIDX_STAT_RXFULL:
	case MSPM0_SPI_IIDX_STAT_END_OF_TRANSMIT:
		spi_mspm0_rx_drain(base, data);
		spi_mspm0_tx_fill(base, data);

		if (!spi_context_tx_len_left(ctx, data->dfs) &&
		    !spi_context_rx_len_left(ctx, data->dfs)) {
			goto exit; /* transfer finished */
		}

		break;

	case MSPM0_SPI_IIDX_STAT_RTOUT:
		spi_mspm0_rx_drain(base, data);

		if (!spi_context_rx_len_left(ctx, data->dfs)) { /* reception complete */
			goto exit;
		} else { /* wait if cs is still active */
			SPI_UPDATE_REG(base + MSPM0_SPI_IMASK, MSPM0_SPI_IMASK_DISABLE,
				       MSPM0_SPI_INT_RTOUT_MASK)
		}
		break;

	case MSPM0_SPI_IIDX_STAT_RXFIFO_OVF:
		isr_status = -EIO;
		goto exit;
	}
	return;

exit:
	sys_write32(MSPM0_SPI_IMASK_DISABLE, base + MSPM0_SPI_IMASK);
	spi_mspm0_finish(dev, isr_status);
}

static void spi_mspm0_transceive_sync(const struct device *dev)
{
	const struct spi_mspm0_config *cfg = dev->config;
	struct spi_mspm0_data *data = dev->data;
	uint32_t base = cfg->base;

	while (data->tf_size || data->frames_left) {

		while (sys_read32(base + MSPM0_SPI_STAT) & MSPM0_SPI_STAT_BUSY_MASK) {
			/* Wait until transfer is completed */
		}
		spi_mspm0_rx_drain(base, data);
		spi_mspm0_tx_fill(base, data);
	}
	spi_mspm0_finish(dev, 0U);
}

static int spi_mspm0_transceive_common(const struct device *dev, const struct spi_config *config,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs, bool asynchronous,
				       spi_callback_t cb, void *userdata)
{
	const struct spi_mspm0_config *cfg = dev->config;
	struct spi_mspm0_data *data = dev->data;
	struct spi_context *ctx = &data->spi_ctx;
	uint32_t base = cfg->base;
	int ret;

	spi_context_lock(ctx, asynchronous, cb, userdata, config);
	ret = spi_mspm0_configure(dev, config);

	if (ret) {
		goto exit;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, data->dfs);
	data->frames_left =
		MAX(spi_context_total_tx_len(ctx), spi_context_total_rx_len(ctx)) / data->dfs;

	if (!data->frames_left) {
		ret = 0; /* zero-length buffer handling */
		goto exit;
	}

	spi_mspm0_buffer_clear(base, data);

	if (!spi_context_is_peripheral(ctx)) { /* controller mode */
		spi_context_cs_control(ctx, true);
		spi_mspm0_tx_fill(base, data);

		if (asynchronous) {
			sys_write32(SPI_MSPM0_IMASK_CONTROLLER_ENABLE, base + MSPM0_SPI_IMASK);
		} else {
			spi_mspm0_transceive_sync(dev);
		}

	} else { /* peripheral mode */
		spi_mspm0_tx_fill(base, data);
		sys_write32(SPI_MSPM0_IMASK_PERIPHERAL_ENABLE, base + MSPM0_SPI_IMASK);
	}

	ret = spi_context_wait_for_completion(ctx);
exit:
	spi_context_release(ctx, ret);
	return ret;
}

static int spi_mspm0_transceive(const struct device *dev, const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	return spi_mspm0_transceive_common(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_mspm0_transceive_async(const struct device *dev, const struct spi_config *config,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				      void *userdata)
{
	return spi_mspm0_transceive_common(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_mspm0_release(const struct device *dev, const struct spi_config *config)
{
	const struct spi_mspm0_config *cfg = dev->config;
	struct spi_mspm0_data *data = dev->data;

	if (!spi_context_configured(&data->spi_ctx, config)) {
		return -EINVAL;
	}

	if (sys_read32(cfg->base + MSPM0_SPI_STAT) & MSPM0_SPI_STAT_BUSY_MASK) {
		return -EBUSY;
	}

	spi_context_unlock_unconditionally(&data->spi_ctx);
	return 0;
}

static DEVICE_API(spi, spi_mspm0_api) = {
	.transceive = spi_mspm0_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_mspm0_transceive_async,
#endif
	.release = spi_mspm0_release,
};

static int spi_mspm0_init(const struct device *dev)
{
	const struct spi_mspm0_config *cfg = dev->config;
	struct spi_mspm0_data *data = dev->data;
	int ret = 0;

	if (!device_is_ready(cfg->clk_dev)) {
		LOG_ERR("Clock device not ready");
		return -ENODEV;
	}

	sys_write32(MSPM0_SPI_PWREN_KEY | MSPM0_SPI_PWREN_ENABLE_MASK, cfg->base + MSPM0_SPI_PWREN);

	/* Small settle delay to power enable */
	ret = WAIT_FOR((sys_read32(cfg->base + MSPM0_SPI_PWREN) & MSPM0_SPI_PWREN_ENABLE_MASK),
		       1000, k_busy_wait(10));
	if (!ret) {
		return -ETIMEDOUT;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret < 0) {
		return ret;
	}

	sys_write32(MSPM0_SPI_ICLR_MASK, cfg->base + MSPM0_SPI_ICLR);

	/* Select the functional clock source given by devicetree, with no division */
	sys_write32(0U, cfg->base + MSPM0_SPI_CLKDIV);
	sys_write32(MSPM0_CLOCK_PERIPH_REG_MASK(cfg->clk_subsys.clk), cfg->base + MSPM0_SPI_CLKSEL);

	sys_write32(MSPM0_SPI_IMASK_DISABLE, cfg->base + MSPM0_SPI_IMASK);

	if (cfg->cs_gpios_present) {
		ret = spi_context_cs_configure_all(&data->spi_ctx);

		if (ret < 0) {
			return ret;
		}
	}

	cfg->irq_config_func(dev);
	spi_context_unlock_unconditionally(&data->spi_ctx);
	return 0;
}

#define SPI_MSPM0_IRQ_FUNC(inst)                                                                   \
	static void spi_mspm0_irq_config_##inst(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), spi_mspm0_isr,        \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

#define MSPM0_SPI_INIT(inst)                                                                       \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	SPI_MSPM0_IRQ_FUNC(inst)                                                                   \
                                                                                                   \
	static const struct spi_mspm0_config spi_mspm0_cfg_##inst = {                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.base = (uint32_t)DT_INST_REG_ADDR(inst),                                          \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                               \
		.clk_subsys = MSPM0_CLOCK_SUBSYS_FN(inst),                                         \
		.irq_config_func = spi_mspm0_irq_config_##inst,                                    \
		.cs_gpios_present = DT_INST_NODE_HAS_PROP(inst, cs_gpios),                         \
	};                                                                                         \
                                                                                                   \
	static struct spi_mspm0_data spi_mspm0_data_##inst = {                                     \
		SPI_CONTEXT_INIT_LOCK(spi_mspm0_data_##inst, spi_ctx),                             \
		SPI_CONTEXT_INIT_SYNC(spi_mspm0_data_##inst, spi_ctx),                             \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), spi_ctx)};                      \
                                                                                                   \
	SPI_DEVICE_DT_INST_DEFINE(inst, spi_mspm0_init, NULL, &spi_mspm0_data_##inst,              \
				  &spi_mspm0_cfg_##inst, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,    \
				  &spi_mspm0_api);

DT_INST_FOREACH_STATUS_OKAY(MSPM0_SPI_INIT)
