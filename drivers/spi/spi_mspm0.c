/*
 * Copyright (c) 2024 Bang & Olufsen A/S, Denmark
 * Copyright (c) 2025 Linumiz GmbH
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

/* TI DriverLib includes */
#include <driverlib/dl_spi.h>

LOG_MODULE_REGISTER(spi_mspm0, CONFIG_SPI_LOG_LEVEL);

/* This must be included after log module registration */
#include "spi_context.h"

#define POWER_STARTUP_DELAY 16

#define SPI_DT_CLK_DIV(inst) DT_INST_PROP(inst, clk_div)

#define SPI_DT_CLK_DIV_ENUM(inst) _CONCAT(DL_SPI_CLOCK_DIVIDE_RATIO_, SPI_DT_CLK_DIV(inst))

#define SPI_MODE(operation) (operation & BIT(0) ? DL_SPI_MODE_PERIPHERAL : DL_SPI_MODE_CONTROLLER)

#define BIT_ORDER_MODE(operation)                                                                  \
	(operation & BIT(4) ? DL_SPI_BIT_ORDER_LSB_FIRST : DL_SPI_BIT_ORDER_MSB_FIRST)

#define DATA_SIZE_MODE(operation) (SPI_WORD_SIZE_GET(operation) - 1)

#define POLARITY_MODE(operation)                                                                   \
	(SPI_MODE_GET(operation) & SPI_MODE_CPOL ? SPI_CTL0_SPO_HIGH : SPI_CTL0_SPO_LOW)

#define PHASE_MODE(operation)                                                                      \
	(SPI_MODE_GET(operation) & SPI_MODE_CPHA ? SPI_CTL0_SPH_SECOND : SPI_CTL0_SPH_FIRST)

#define DUPLEX_MODE(operation)                                                                     \
	(operation & BIT(11) ? SPI_CTL0_FRF_MOTOROLA_3WIRE : SPI_CTL0_FRF_MOTOROLA_4WIRE)

/* Only motorola format requires config - TI format is a single value */
#define FRAME_FORMAT_MODE(operation)                                                               \
	(operation & SPI_FRAME_FORMAT_TI                                                           \
		 ? SPI_CTL0_FRF_TI_SYNC                                                            \
		 : DUPLEX_MODE(operation) | POLARITY_MODE(operation) | PHASE_MODE(operation))

/* 0x38 represents the bits 8, 16 and 32. Knowing that 24 is bits 8 and 16
 * These are the bits were when you divide by 8, you keep the result as it is.
 * For all the other ones, 4 to 7, 9 to 15, etc... you need a +1,
 * since on such division it takes only the result above 0
 */
#define BYTES_PER_FRAME(word_size)                                                                 \
	(((word_size) & ~0x38) ? (((word_size) / 8) + 1) : ((word_size) / 8))

struct spi_mspm0_config {
	SPI_Regs *base;
	uint32_t clock_frequency;
	const struct pinctrl_dev_config *pinctrl;
	const DL_SPI_ClockConfig clock_config;
};

struct spi_mspm0_data {
	struct spi_context ctx;
	uint8_t dfs;
};

static int spi_mspm0_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
	const struct spi_mspm0_config *const cfg = dev->config;
	struct spi_mspm0_data *const data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint16_t clock_scr;

	if (spi_context_configured(ctx, spi_cfg)) {
		/* This configuration is already in use */
		return 0;
	}

	/* Only master mode is supported */
	if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_MASTER) {
		return -ENOTSUP;
	}

	if (spi_cfg->frequency > (cfg->clock_frequency / 2)) {
		return -EINVAL;
	}

	/* See DL_SPI_setBitRateSerialClockDivider for details */
	clock_scr = (cfg->clock_frequency / (2 * spi_cfg->frequency)) - 1;

	if (!IN_RANGE(clock_scr, 0, 1023)) {
		return -EINVAL;
	}

	const DL_SPI_Config dl_cfg = {
		.parity = DL_SPI_PARITY_NONE,			/* Currently unused in zephyr */
		.chipSelectPin = DL_SPI_CHIP_SELECT_NONE,	/* spi_context controls the CS */
		.mode = SPI_MODE(spi_cfg->operation),
		.bitOrder = BIT_ORDER_MODE(spi_cfg->operation),
		.dataSize = DATA_SIZE_MODE(spi_cfg->operation),
		.frameFormat = FRAME_FORMAT_MODE(spi_cfg->operation),
	};

	/* Peripheral should always be disabled prior to applying a new configuration */
	DL_SPI_disable(cfg->base);
	DL_SPI_init(cfg->base, (DL_SPI_Config *)&dl_cfg);
	DL_SPI_setBitRateSerialClockDivider(cfg->base, (uint32_t)clock_scr);

	data->dfs = BYTES_PER_FRAME(SPI_WORD_SIZE_GET(spi_cfg->operation));
	if (data->dfs > 2) {
		DL_SPI_enablePacking(cfg->base);
	} else {
		DL_SPI_disablePacking(cfg->base);
	}

	if (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_LOOP) {
		DL_SPI_enableLoopbackMode(cfg->base);
	} else {
		DL_SPI_disableLoopbackMode(cfg->base);
	}

	DL_SPI_enable(cfg->base);
	if (!DL_SPI_isEnabled(cfg->base)) {
		return -EAGAIN;
	}

	/* Save config so it can be reused, it's required for the lock owner to work */
	ctx->config = spi_cfg;

	return 0;
}

static void spi_mspm0_frame_tx(const struct device *dev)
{
	const struct spi_mspm0_config *cfg = dev->config;
	struct spi_mspm0_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	/* NOP TX if no data is expected */
	uint32_t tx_frame = 0;

	if (spi_context_tx_buf_on(ctx)) {
		if (data->dfs == 1) {
			tx_frame = UNALIGNED_GET((uint8_t *)(ctx->tx_buf));
			DL_SPI_transmitDataCheck8(cfg->base, (uint8_t)tx_frame);
		} else if (data->dfs == 2) {
			tx_frame = UNALIGNED_GET((uint16_t *)(ctx->tx_buf));
			DL_SPI_transmitDataCheck16(cfg->base, (uint16_t)tx_frame);
		} else {
			tx_frame = UNALIGNED_GET((uint32_t *)(ctx->tx_buf));
			DL_SPI_transmitDataCheck32(cfg->base, tx_frame);
		}
	}

	while (DL_SPI_isBusy(cfg->base)) {
		/* Wait for tx fifo to be sent */
	}
	spi_context_update_tx(ctx, data->dfs, 1);
}

static void spi_mspm0_frame_rx(const struct device *dev)
{
	const struct spi_mspm0_config *cfg = dev->config;
	struct spi_mspm0_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t rx_val = 0;

	if (data->dfs == 1) {
		if (DL_SPI_receiveDataCheck8(cfg->base, (uint8_t *)&rx_val)) {
			UNALIGNED_PUT((uint8_t)rx_val, (uint8_t *)ctx->rx_buf);
		}
	} else if (data->dfs == 2) {
		if (DL_SPI_receiveDataCheck16(cfg->base, (uint16_t *)&rx_val)) {
			UNALIGNED_PUT((uint16_t)rx_val, (uint16_t *)ctx->rx_buf);
		}
	} else {
		if (DL_SPI_receiveDataCheck32(cfg->base, &rx_val)) {
			UNALIGNED_PUT(rx_val, (uint32_t *)ctx->rx_buf);
		}
	}

	spi_context_update_rx(ctx, data->dfs, 1);
}

static void spi_mspm0_start_transfer(const struct device *dev, const struct spi_config *spi_cfg)
{
	struct spi_mspm0_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	spi_context_cs_control(ctx, true);

	while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx)) {
		spi_mspm0_frame_tx(dev);
		spi_mspm0_frame_rx(dev);
	}

	spi_context_cs_control(ctx, false);
	spi_context_complete(ctx, dev, 0);
}

static int spi_mspm0_transceive(const struct device *dev,
				const struct spi_config *spi_cfg,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	struct spi_mspm0_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

	spi_context_lock(ctx, false, NULL, NULL, spi_cfg);

	ret = spi_mspm0_configure(dev, spi_cfg);
	if (ret != 0) {
		spi_context_release(ctx, ret);
		return ret;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, data->dfs);

	spi_mspm0_start_transfer(dev, spi_cfg);

	ret = spi_context_wait_for_completion(ctx);
	spi_context_release(ctx, ret);

	return ret;
}

static int spi_mspm0_release(const struct device *dev, const struct spi_config *config)
{
	const struct spi_mspm0_config *cfg = dev->config;
	struct spi_mspm0_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	if (!spi_context_configured(ctx, config)) {
		return -EINVAL;
	}

	if (DL_SPI_isBusy(cfg->base)) {
		return -EBUSY;
	}

	spi_context_unlock_unconditionally(ctx);
	return 0;
}

static const struct spi_driver_api spi_mspm0_api = {
	.transceive = spi_mspm0_transceive,
	.release    = spi_mspm0_release,
};

static int spi_mspm0_init(const struct device *dev)
{
	const struct spi_mspm0_config *cfg = dev->config;
	struct spi_mspm0_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	DL_SPI_enablePower(cfg->base);

	delay_cycles(POWER_STARTUP_DELAY);

	ret = pinctrl_apply_state(cfg->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ret = spi_context_cs_configure_all(ctx);
	if (ret < 0) {
		return ret;
	}

	DL_SPI_setClockConfig(cfg->base, (DL_SPI_ClockConfig *)&cfg->clock_config);
	DL_SPI_enable(cfg->base);

	spi_context_unlock_unconditionally(ctx);
	return ret;
}

#define MSPM0_SPI_INIT(inst)									\
	PINCTRL_DT_INST_DEFINE(inst);								\
												\
	static struct spi_mspm0_config spi_mspm0_##inst##_cfg = {				\
		.base = (SPI_Regs *)DT_INST_REG_ADDR(inst),					\
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),				\
		.clock_config = {.clockSel =							\
				  MSPM0_CLOCK_PERIPH_REG_MASK(DT_INST_CLOCKS_CELL(inst, clk)),  \
				 .divideRatio = SPI_DT_CLK_DIV_ENUM(inst)},			\
		.clock_frequency = DT_PROP(DT_INST_CLOCKS_CTLR(inst), clock_frequency),		\
	};											\
												\
	static struct spi_mspm0_data spi_mspm0_##inst##_data = {				\
		SPI_CONTEXT_INIT_LOCK(spi_mspm0_##inst##_data, ctx),				\
		SPI_CONTEXT_INIT_SYNC(spi_mspm0_##inst##_data, ctx),				\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)};			\
												\
	DEVICE_DT_INST_DEFINE(inst, spi_mspm0_init, NULL, &spi_mspm0_##inst##_data,		\
			      &spi_mspm0_##inst##_cfg, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,	\
			      &spi_mspm0_api);

DT_INST_FOREACH_STATUS_OKAY(MSPM0_SPI_INIT)
