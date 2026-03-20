/* Copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_cat1_i2s

#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <cy_i2s.h>

LOG_MODULE_REGISTER(i2s_ifx_cat1, CONFIG_I2S_LOG_LEVEL);

#define TX_QUEUE_SIZE  CONFIG_I2S_INFINEON_TX_QUEUE_SIZE
#define RX_QUEUE_SIZE  CONFIG_I2S_INFINEON_RX_QUEUE_SIZE

#define INTR_TX_ERRORS  (CY_I2S_INTR_TX_OVERFLOW | CY_I2S_INTR_TX_UNDERFLOW)
#define INTR_RX_ERRORS  (CY_I2S_INTR_RX_OVERFLOW | CY_I2S_INTR_RX_UNDERFLOW)
#define TX_MAX_BLOCK_SAMPLES  170U
#define RX_MAX_BLOCK_SAMPLES  255U

#define SF_XFER_PENDING  BIT(0)
#define SF_LAST_BLOCK    BIT(1)
#define SF_DRAIN         BIT(2)
#define SF_WAIT_START    BIT(3)

struct queue_item {
	void   *buffer;
	size_t  size;
};

struct i2s_stream {
	struct i2s_config cfg;
	struct k_msgq     queue;
	void             *mem_block;
	uint8_t           state;
	uint8_t           flags;
};

struct dma_channel {
	const struct device    *dev_dma;
	uint32_t                channel_num;
	struct dma_config       dma_cfg;
	struct dma_block_config blk_cfg;
};

struct ifx_i2s_data {
	struct i2s_stream   tx;
	struct i2s_stream   rx;
	struct dma_channel  dma_tx;
	struct dma_channel  dma_rx;
	struct queue_item   tx_queue_buf[TX_QUEUE_SIZE];
	struct queue_item   rx_queue_buf[RX_QUEUE_SIZE];
};

struct ifx_i2s_config {
	I2S_Type                        *reg;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(const struct device *dev);
};

static int  start_dma_tx_transfer(const struct device *dev);
static int  start_dma_rx_transfer(const struct device *dev);
static void i2s_tx_stream_disable(const struct device *dev, bool drop);
static void i2s_rx_stream_disable(const struct device *dev, bool drop);

static cy_en_i2s_len_t word_size_to_pdl_len(uint8_t ws)
{
	switch (ws) {
	case 8:
		return CY_I2S_LEN8;
	case 16:
		return CY_I2S_LEN16;
	case 18:
		return CY_I2S_LEN18;
	case 20:
		return CY_I2S_LEN20;
	case 24:
		return CY_I2S_LEN24;
	default:
		return CY_I2S_LEN32;
	}
}

static uint32_t word_size_to_dma_bytes(uint8_t ws)
{
	if (ws <= 8U) {
	       	return 1U;
	} else if (ws <= 16U) {
	       	return 2U;
	}
	return 4U;
}

static int compute_clk_div(const struct i2s_config *cfg, uint8_t *out)
{
	uint32_t sck = cfg->frame_clk_freq * (uint32_t)cfg->channels * 32U;
	uint32_t clock, div;

	clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(clk_hf5)),
			       NULL, &clock);
	if (sck == 0U) {
		return -EINVAL;
	}

	div = clock / (sck * 8U);
	if (div < 1U || div > 64U) {
		LOG_ERR("Clk div out of range: %u", div);
		return -EINVAL;
	}
	*out = (uint8_t)div;
	return 0;
}

static void queue_flush(struct i2s_stream *s)
{
	struct queue_item item;
	while (k_msgq_get(&s->queue, &item, K_NO_WAIT) == 0) {
		k_mem_slab_free(s->cfg.mem_slab, item.buffer);
	}
}

static int start_dma_tx_transfer(const struct device *dev)
{
	struct ifx_i2s_data *data = dev->data;
	const struct ifx_i2s_config *cfg = dev->config;
	struct i2s_stream *stream = &data->tx;
	struct dma_channel *dma = &data->dma_tx;
	struct queue_item item;
	int ret;

	ret = k_msgq_get(&stream->queue, &item, K_NO_WAIT);
	if (ret != 0) {
		if (stream->state == I2S_STATE_STOPPING)
			stream->flags |= SF_LAST_BLOCK | SF_DRAIN;
		for (int i = 0; i < 6; i++)
			Cy_I2S_WriteTxData(cfg->reg, 0U);
		return ret;
	}

	stream->mem_block = item.buffer;
	dma->blk_cfg.source_address = (uint32_t)item.buffer;
	dma->blk_cfg.block_size = (uint32_t)item.size / dma->dma_cfg.source_data_size;

	ret = dma_config(dma->dev_dma, dma->channel_num, &dma->dma_cfg);
	if (ret < 0)
		goto fail;
	ret = dma_start(dma->dev_dma, dma->channel_num);
	if (ret < 0)
		goto fail;
	return 0;
fail:
	k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
	stream->mem_block = NULL;
	return ret;
}

static int start_dma_rx_transfer(const struct device *dev)
{
	struct ifx_i2s_data *data = dev->data;
	const struct ifx_i2s_config *cfg = dev->config;
	struct i2s_stream *stream = &data->rx;
	struct dma_channel *dma = &data->dma_rx;
	int ret;

	ret = k_mem_slab_alloc(stream->cfg.mem_slab,
			       &stream->mem_block, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("RX: no free slab block");
		i2s_rx_stream_disable(dev, false);
		stream->state = I2S_STATE_ERROR;
		return ret;
	}

	dma->blk_cfg.dest_address = (uint32_t)stream->mem_block;
	dma->blk_cfg.block_size = (uint32_t)stream->cfg.block_size /
				   dma->dma_cfg.source_data_size;

	ret = dma_config(dma->dev_dma, dma->channel_num, &dma->dma_cfg);
	if (ret < 0) {
	       	goto fail;
	}
	ret = dma_start(dma->dev_dma, dma->channel_num);
	if (ret < 0) {
		goto fail;
	}

	Cy_I2S_SetInterruptMask(cfg->reg,
		Cy_I2S_GetInterruptMask(cfg->reg) & ~CY_I2S_INTR_RX_TRIGGER);
	return 0;
fail:
	k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
	stream->mem_block = NULL;
	return ret;
}

static int i2s_tx_stream_start(const struct device *dev)
{
	struct ifx_i2s_data *data = dev->data;
	int ret;

	data->tx.flags |= SF_WAIT_START;
	ret = start_dma_tx_transfer(dev);
	if (ret != 0) {
		data->tx.flags &= ~SF_WAIT_START;
		LOG_ERR("TX start failed: %d", ret);
	}
	return ret;
}

static int i2s_rx_stream_start(const struct device *dev)
{
	const struct ifx_i2s_config *cfg = dev->config;

	Cy_I2S_SetInterruptMask(cfg->reg,
		Cy_I2S_GetInterruptMask(cfg->reg) |
		CY_I2S_INTR_RX_TRIGGER | INTR_RX_ERRORS);
	Cy_I2S_EnableRx(cfg->reg);
	return 0;
}

static void i2s_tx_stream_disable(const struct device *dev, bool drop)
{
	const struct ifx_i2s_config *cfg = dev->config;
	struct ifx_i2s_data *data = dev->data;
	struct i2s_stream *s = &data->tx;

	Cy_I2S_SetInterruptMask(cfg->reg,
		Cy_I2S_GetInterruptMask(cfg->reg) & ~(INTR_TX_ERRORS | CY_I2S_INTR_TX_TRIGGER));
	Cy_I2S_DisableTx(cfg->reg);
	dma_stop(data->dma_tx.dev_dma, data->dma_tx.channel_num);
	if (s->mem_block) {
		k_mem_slab_free(s->cfg.mem_slab, s->mem_block);
		s->mem_block = NULL;
	}
	if (drop) {
		queue_flush(s);
	}
}

static void i2s_rx_stream_disable(const struct device *dev, bool drop)
{
	const struct ifx_i2s_config *cfg = dev->config;
	struct ifx_i2s_data *data = dev->data;
	struct i2s_stream *s = &data->rx;

	Cy_I2S_SetInterruptMask(cfg->reg,
		Cy_I2S_GetInterruptMask(cfg->reg) & ~(INTR_RX_ERRORS | CY_I2S_INTR_RX_TRIGGER));
	Cy_I2S_DisableRx(cfg->reg);
	dma_stop(data->dma_rx.dev_dma, data->dma_rx.channel_num);
	if (s->mem_block) {
		k_mem_slab_free(s->cfg.mem_slab, s->mem_block);
		s->mem_block = NULL;
	}
	if (drop) {
		queue_flush(s);
		Cy_I2S_ClearRxFifo(cfg->reg);
	}
}

static void dma_tx_callback(const struct device *dma_dev, void *arg,
			    uint32_t channel, int status)
{
	const struct device *dev = arg;
	struct ifx_i2s_data *data = dev->data;
	const struct ifx_i2s_config *cfg = dev->config;
	struct i2s_stream *s = &data->tx;
	uint32_t mask;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		LOG_ERR("TX DMA error %d", status);
		if (s->mem_block) {
			k_mem_slab_free(s->cfg.mem_slab, s->mem_block);
			s->mem_block = NULL;
		}
		s->state = I2S_STATE_ERROR;
		return;
	}

	k_mem_slab_free(s->cfg.mem_slab, s->mem_block);
	s->mem_block = NULL;

	Cy_I2S_ClearInterrupt(cfg->reg, CY_I2S_INTR_TX_TRIGGER);
	mask = Cy_I2S_GetInterruptMask(cfg->reg);
	Cy_I2S_SetInterruptMask(cfg->reg, mask | CY_I2S_INTR_TX_TRIGGER);

	if (s->flags & SF_XFER_PENDING) {
		s->flags &= ~SF_XFER_PENDING;
		(void)start_dma_tx_transfer(dev);
	}

	if (s->flags & SF_WAIT_START) {
		s->flags &= ~SF_WAIT_START;
		Cy_I2S_ClearInterrupt(cfg->reg,
				      CY_I2S_INTR_TX_TRIGGER | INTR_TX_ERRORS);
		mask = Cy_I2S_GetInterruptMask(cfg->reg);
		Cy_I2S_SetInterruptMask(cfg->reg,
					mask | CY_I2S_INTR_TX_TRIGGER | INTR_TX_ERRORS);
		Cy_I2S_EnableTx(cfg->reg);
	}
}

static void dma_rx_callback(const struct device *dma_dev, void *arg,
			    uint32_t channel, int status)
{
	const struct device *dev = arg;
	struct ifx_i2s_data *data = dev->data;
	const struct ifx_i2s_config *cfg = dev->config;
	struct i2s_stream *s = &data->rx;
	struct queue_item item;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		LOG_ERR("RX DMA error %d", status);
		k_mem_slab_free(s->cfg.mem_slab, s->mem_block);
		s->mem_block = NULL;
		s->state = I2S_STATE_ERROR;
		return;
	}

	item.buffer = s->mem_block;
	item.size   = s->cfg.block_size;
	s->mem_block = NULL;

	if (k_msgq_put(&s->queue, &item, K_NO_WAIT) != 0) {
		LOG_ERR("RX queue full, dropping block");
		k_mem_slab_free(s->cfg.mem_slab, item.buffer);
		s->state = I2S_STATE_ERROR;
		return;
	}

	if (s->flags & SF_LAST_BLOCK) {
		i2s_rx_stream_disable(dev, false);
		s->state = I2S_STATE_READY;
		return;
	}

	if (s->flags & SF_XFER_PENDING) {
		s->flags &= ~SF_XFER_PENDING;
		(void)start_dma_rx_transfer(dev);
		return;
	}

	Cy_I2S_SetInterruptMask(cfg->reg,
				Cy_I2S_GetInterruptMask(cfg->reg) | CY_I2S_INTR_RX_TRIGGER);
}

static void tx_fifo_trigger_handler(const struct device *dev)
{
	struct ifx_i2s_data *data = dev->data;
	const struct ifx_i2s_config *cfg = dev->config;
	struct i2s_stream *s = &data->tx;

	switch (s->state) {
	case I2S_STATE_RUNNING:
	case I2S_STATE_STOPPING:
		Cy_I2S_SetInterruptMask(cfg->reg,
			Cy_I2S_GetInterruptMask(cfg->reg) &
			~CY_I2S_INTR_TX_TRIGGER);

		if (s->mem_block == NULL) {
			if (s->flags & SF_LAST_BLOCK) {
				for (int i = 0; i < 4; i++)
					Cy_I2S_WriteTxData(cfg->reg, 0U);
				s->flags |= SF_DRAIN;
			} else {
				(void)start_dma_tx_transfer(dev);
			}
		} else {
			s->flags |= SF_XFER_PENDING;
		}
		break;
	case I2S_STATE_ERROR:
		i2s_tx_stream_disable(dev, false);
		break;
	default:
		LOG_ERR("TX trigger: unhandled state %d", s->state);
		break;
	}
}

static void rx_fifo_trigger_handler(const struct device *dev)
{
	struct ifx_i2s_data *data = dev->data;
	struct i2s_stream *s = &data->rx;

	switch (s->state) {
	case I2S_STATE_RUNNING:
	case I2S_STATE_STOPPING:
		if (s->mem_block == NULL) {
			(void)start_dma_rx_transfer(dev);
		} else {
			s->flags |= SF_XFER_PENDING;
		}
		break;
	case I2S_STATE_ERROR:
		i2s_rx_stream_disable(dev, false);
		break;
	default:
		LOG_ERR("RX trigger: unhandled state %d", s->state);
		break;
	}
}

static void i2s_isr(const struct device *dev)
{
	const struct ifx_i2s_config *cfg = dev->config;
	struct ifx_i2s_data *data = dev->data;
	uint32_t intr = Cy_I2S_GetInterruptStatusMasked(cfg->reg);

	if (intr & CY_I2S_INTR_TX_OVERFLOW) {
		LOG_ERR("TX overflow");
		data->tx.state = I2S_STATE_ERROR;
	}

	if (intr & CY_I2S_INTR_TX_UNDERFLOW) {
		i2s_tx_stream_disable(dev, false);
		data->tx.state =
			((data->tx.flags & (SF_LAST_BLOCK | SF_DRAIN)) ==
			 (SF_LAST_BLOCK | SF_DRAIN))
			? I2S_STATE_READY : I2S_STATE_ERROR;
	}

	if (intr & CY_I2S_INTR_TX_TRIGGER) {
		tx_fifo_trigger_handler(dev);
	}

	if (intr & CY_I2S_INTR_RX_OVERFLOW) {
		 LOG_ERR("RX overflow");
		 data->rx.state = I2S_STATE_ERROR;
	}

	if (intr & CY_I2S_INTR_RX_UNDERFLOW) {
		LOG_ERR("RX underflow");
		data->rx.state = I2S_STATE_ERROR;
	}

	if (intr & CY_I2S_INTR_RX_TRIGGER) {
		if (data->rx.state == I2S_STATE_STOPPING) {
			Cy_I2S_DisableRx(cfg->reg);
		}
		rx_fifo_trigger_handler(dev);
	}

	Cy_I2S_ClearInterrupt(cfg->reg, intr);
}

static int ifx_i2s_configure(const struct device *dev, enum i2s_dir dir,
			     const struct i2s_config *i2s_cfg)
{
	const struct ifx_i2s_config *cfg = dev->config;
	struct ifx_i2s_data *data = dev->data;
	cy_stc_i2s_config_t pdl = {0};
	struct i2s_stream *stream;
	cy_en_i2s_alignment_t align;
	cy_en_i2s_len_t wlen;
	uint32_t dma_bytes, blk_samp;
	uint8_t clk_div;
	bool master, clk_inv;
	int ret;

	if (dir == I2S_DIR_BOTH) {
		return -ENOSYS;
	}

	stream = (dir == I2S_DIR_TX) ? &data->tx : &data->rx;

	if (stream->state != I2S_STATE_NOT_READY &&
	    stream->state != I2S_STATE_READY) {
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		stream->state = I2S_STATE_NOT_READY;
		return 0;
	}

	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		align = CY_I2S_I2S_MODE;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		align = CY_I2S_LEFT_JUSTIFIED;
		break;
	default:
		return -EINVAL;
	}

	if ((i2s_cfg->format & I2S_FMT_DATA_ORDER_LSB) ||
	    (i2s_cfg->options & I2S_OPT_BIT_CLK_GATED) ||
	    (i2s_cfg->options & I2S_OPT_PINGPONG)) {
		return -ENOTSUP;
	}

	if (!!(i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) !=
	    !!(i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE)) {
		return -EINVAL;
	}

	if (i2s_cfg->channels != 2U) {
		return -EINVAL;
	}

	master    = !(i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE);
	wlen      = word_size_to_pdl_len(i2s_cfg->word_size);
	dma_bytes = word_size_to_dma_bytes(i2s_cfg->word_size);
	blk_samp  = (uint32_t)i2s_cfg->block_size / dma_bytes;
	clk_inv   = !!(i2s_cfg->format & I2S_FMT_BIT_CLK_INV);

	if (dir == I2S_DIR_TX && blk_samp > TX_MAX_BLOCK_SAMPLES) {
		return -EINVAL;
	}

	if (dir == I2S_DIR_RX && blk_samp > RX_MAX_BLOCK_SAMPLES) {
		return -EINVAL;
	}

	if (master) {
		ret = compute_clk_div(i2s_cfg, &clk_div);
		if (ret) return ret;
	} else {
		clk_div = 2U;
	}

	pdl.clkDiv  = clk_div;
	pdl.mclkDiv = CY_I2S_MCLK_DIV_8;
	pdl.mclkEn  = true;

	if (dir == I2S_DIR_TX) {
		pdl.txEnabled          = true;
		pdl.txMasterMode       = master;
		pdl.txAlignment        = align;
		pdl.txWsPulseWidth     = CY_I2S_WS_ONE_CHANNEL_LENGTH;
		pdl.txSckoInversion    = clk_inv;
		pdl.txSckiInversion    = clk_inv;
		pdl.txChannels         = 2U;
		pdl.txChannelLength    = CY_I2S_LEN32;
		pdl.txWordLength       = wlen;
		pdl.txOverheadValue    = CY_I2S_OVHDATA_ZERO;
		pdl.txFifoTriggerLevel = (uint8_t)(blk_samp / 2U);
		data->dma_tx.dma_cfg.source_data_size = dma_bytes;
		data->dma_tx.dma_cfg.dest_data_size   = dma_bytes;
	} else {
		pdl.rxEnabled          = true;
		pdl.rxMasterMode       = master;
		pdl.rxAlignment        = align;
		pdl.rxWsPulseWidth     = CY_I2S_WS_ONE_CHANNEL_LENGTH;
		pdl.rxSckoInversion    = clk_inv;
		pdl.rxSckiInversion    = clk_inv;
		pdl.rxChannels         = 2U;
		pdl.rxChannelLength    = CY_I2S_LEN32;
		pdl.rxWordLength       = wlen;
		pdl.rxFifoTriggerLevel = (uint8_t)(blk_samp - 1U);
		data->dma_rx.dma_cfg.source_data_size = dma_bytes;
		data->dma_rx.dma_cfg.dest_data_size   = dma_bytes;
	}

	if (CY_I2S_SUCCESS != Cy_I2S_Init(cfg->reg, &pdl)) {
		LOG_ERR("Cy_I2S_Init failed");
		return -EIO;
	}
	Cy_I2S_SetInterruptMask(cfg->reg, INTR_TX_ERRORS | INTR_RX_ERRORS);

	queue_flush(stream);
	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));
	stream->state = I2S_STATE_READY;
	return 0;
}

static const struct i2s_config *ifx_i2s_config_get(const struct device *dev,
						   enum i2s_dir dir)
{
	struct ifx_i2s_data *data = dev->data;
	struct i2s_stream *s = (dir == I2S_DIR_TX) ? &data->tx : &data->rx;
	return (s->state != I2S_STATE_NOT_READY) ? &s->cfg : NULL;
}

static int ifx_i2s_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_stream *s = &((struct ifx_i2s_data *)dev->data)->rx;
	struct queue_item item;
	int ret;

	if (s->state == I2S_STATE_NOT_READY) {
	       	return -EIO;
	}
	ret = k_msgq_get(&s->queue, &item, SYS_TIMEOUT_MS(s->cfg.timeout));
	if (ret != 0) {
		return (s->state == I2S_STATE_ERROR) ? -EIO : ret;
	}
	*mem_block = item.buffer;
	*size = item.size;
	return 0;
}

static int ifx_i2s_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_stream *s = &((struct ifx_i2s_data *)dev->data)->tx;
	struct queue_item item = { .buffer = mem_block, .size = size };
	int ret;

	if (s->state != I2S_STATE_RUNNING && s->state != I2S_STATE_READY) {
		return -EIO;
	}

	ret = k_msgq_put(&s->queue, &item, SYS_TIMEOUT_MS(s->cfg.timeout));
	if (ret) {
		 LOG_ERR("k_msgq_put failed %d", ret);
	}
	return ret;
}

static int ifx_i2s_trigger(const struct device *dev, enum i2s_dir dir,
			   enum i2s_trigger_cmd cmd)
{
	struct ifx_i2s_data *data = dev->data;
	struct i2s_stream *s;
	unsigned int key;
	int ret = 0;

	if (dir == I2S_DIR_BOTH) {
		return -ENOSYS;
	}
	s = (dir == I2S_DIR_TX) ? &data->tx : &data->rx;
	key = irq_lock();

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (s->state != I2S_STATE_READY) {
			ret = -EIO;
			break;
		}
		s->flags = 0;
		ret = (dir == I2S_DIR_TX) ? i2s_tx_stream_start(dev)
					  : i2s_rx_stream_start(dev);
		if (ret == 0) {
			s->state = I2S_STATE_RUNNING;
		}
		break;
	case I2S_TRIGGER_STOP:
		if (s->state != I2S_STATE_RUNNING) {
		       	ret = -EIO;
			break;
	       	}
		s->flags |= SF_LAST_BLOCK | SF_DRAIN;
		s->state = I2S_STATE_STOPPING;
		break;
	case I2S_TRIGGER_DRAIN:
		if (s->state != I2S_STATE_RUNNING) {
			ret = -EIO;
			break;
		}
		s->flags |= (dir == I2S_DIR_TX) ? SF_DRAIN : SF_LAST_BLOCK;
		s->state = I2S_STATE_STOPPING;
		break;
	case I2S_TRIGGER_DROP:
		if (s->state == I2S_STATE_NOT_READY) {
			ret = -EIO;
		       	break;
	       	}
		(dir == I2S_DIR_TX) ? i2s_tx_stream_disable(dev, true)
				    : i2s_rx_stream_disable(dev, true);
		s->state = I2S_STATE_READY;
		break;
	case I2S_TRIGGER_PREPARE:
		if (s->state != I2S_STATE_ERROR) {
			ret = -EIO;
			break;
		}
		(dir == I2S_DIR_TX) ? i2s_tx_stream_disable(dev, true)
				    : i2s_rx_stream_disable(dev, true);
		s->state = I2S_STATE_READY;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	irq_unlock(key);
	return ret;
}

static int ifx_i2s_init(const struct device *dev)
{
	const struct ifx_i2s_config *cfg = dev->config;
	struct ifx_i2s_data *data = dev->data;
	int ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) return ret;

	if (!device_is_ready(data->dma_tx.dev_dma) ||
	    !device_is_ready(data->dma_rx.dev_dma)) {
		  LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	k_msgq_init(&data->tx.queue, (char *)data->tx_queue_buf,
		    sizeof(struct queue_item), TX_QUEUE_SIZE);
	k_msgq_init(&data->rx.queue, (char *)data->rx_queue_buf,
		    sizeof(struct queue_item), RX_QUEUE_SIZE);

	data->dma_tx.blk_cfg.dest_address    = (uint32_t)&cfg->reg->TX_FIFO_WR;
	data->dma_tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->dma_tx.blk_cfg.dest_addr_adj   = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_tx.dma_cfg.head_block      = &data->dma_tx.blk_cfg;
	data->dma_tx.dma_cfg.user_data       = (void *)dev;
	data->dma_tx.dma_cfg.dma_callback    = dma_tx_callback;

	data->dma_rx.blk_cfg.source_address  = (uint32_t)&cfg->reg->RX_FIFO_RD;
	data->dma_rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_rx.blk_cfg.dest_addr_adj   = DMA_ADDR_ADJ_INCREMENT;
	data->dma_rx.dma_cfg.head_block      = &data->dma_rx.blk_cfg;
	data->dma_rx.dma_cfg.user_data       = (void *)dev;
	data->dma_rx.dma_cfg.dma_callback    = dma_rx_callback;

	data->tx.state = I2S_STATE_NOT_READY;
	data->rx.state = I2S_STATE_NOT_READY;

	cfg->irq_config(dev);
	Cy_I2S_SetInterruptMask(cfg->reg, 0);
	LOG_INF("I2S %s initialized", dev->name);
	return 0;
}

static DEVICE_API(i2s, ifx_i2s_api) = {
	.configure  = ifx_i2s_configure,
	.config_get = ifx_i2s_config_get,
	.read       = ifx_i2s_read,
	.write      = ifx_i2s_write,
	.trigger    = ifx_i2s_trigger,
};

#define I2S_DMA_CH(idx, dir, ch_dir)                                       \
	.dev_dma     = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, dir)),\
	.channel_num = DT_INST_DMAS_CELL_BY_NAME(idx, dir, channel),      \
	.dma_cfg = { .channel_direction = ch_dir, .block_count = 1,        \
		     .complete_callback_en = 1, .source_handshake = 1 }

#define IFX_I2S_INIT(n)                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                         \
	static void ifx_i2s_irq_##n(const struct device *dev) {            \
		enable_sys_int(                                            \
			DT_INST_PROP_BY_IDX(n, system_interrupts, 0),      \
			DT_INST_PROP_BY_IDX(n, system_interrupts, 1),      \
			(void (*)(const void *))i2s_isr, dev);             \
	}                                                                  \
	static struct ifx_i2s_data i2s_data_##n = {                        \
		.dma_tx = {I2S_DMA_CH(n, tx, MEMORY_TO_PERIPHERAL)},      \
		.dma_rx = {I2S_DMA_CH(n, rx, PERIPHERAL_TO_MEMORY)},      \
	};                                                                \
	static const struct ifx_i2s_config i2s_cfg_##n = {                \
		.reg        = (I2S_Type *)DT_INST_REG_ADDR(n),            \
		.pcfg       = PINCTRL_DT_INST_DEV_CONFIG_GET(n),          \
		.irq_config = ifx_i2s_irq_##n,                            \
	};                                                                \
	DEVICE_DT_INST_DEFINE(n, &ifx_i2s_init, NULL,                     \
			      &i2s_data_##n, &i2s_cfg_##n,                \
			      POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,      \
			      &ifx_i2s_api);

DT_INST_FOREACH_STATUS_OKAY(IFX_I2S_INIT)
