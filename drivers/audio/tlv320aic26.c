/*
 * Copyright(c), Linumiz 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tlv320aic26

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/audio/codec.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include "tlv320aic26.h"

LOG_MODULE_REGISTER(tlv320aic26, CONFIG_AUDIO_CODEC_LOG_LEVEL);

struct tlv320aic26_config {
	struct spi_dt_spec spi;
	uint32_t mclk_freq;
};

struct tlv320aic26_data {
	audio_route_t route;
	bool configured;
};

struct pll_cfg {
	uint16_t d;
	uint8_t p;
	uint8_t j;
	uint8_t q;
	uint8_t fs_div;
	bool pll_en;
	bool fsref_44k1;
};

static int aic26_write(const struct device *dev, uint8_t page,
		       uint8_t addr, uint16_t val)
{
	const struct tlv320aic26_config *cfg = dev->config;
	uint16_t cmd = AIC26_BUILD_CMD(AIC26_CMD_WRITE, page, addr);
	uint8_t buf[4] = {cmd >> 8, cmd & 0xFF, val >> 8, val & 0xFF};
	const struct spi_buf tx = { .buf = buf, .len = 4 };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	int ret;
	ret = spi_write_dt(&cfg->spi, &tx_set);
	if (ret < 0) {
		 LOG_ERR("SPI write fail P%u R0x%02x: %d", page, addr, ret);
	}
	return ret;
}

static int aic26_read(const struct device *dev, uint8_t page,
		      uint8_t addr, uint16_t *val)
{
	const struct tlv320aic26_config *cfg = dev->config;
	uint16_t cmd = AIC26_BUILD_CMD(AIC26_CMD_READ, page, addr);
	uint8_t tx_buf[4] = {cmd >> 8, cmd & 0xFF, 0, 0};
	uint8_t rx_buf[4] = {0};
	const struct spi_buf tx = { .buf = tx_buf, .len = 4 };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf rx = { .buf = rx_buf, .len = 4 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };
	int ret;

	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI read fail P%u R0x%02x: %d", page, addr, ret);
		return ret;
	}
	*val = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
	return ret;
}

static int aic26_update(const struct device *dev, uint8_t page,
			uint8_t addr, uint16_t mask, uint16_t val)
{
	uint16_t reg;
	int ret = aic26_read(dev, page, addr, &reg);

	if (ret < 0) {
		return ret;
	}
	return aic26_write(dev, page, addr, (reg & ~mask) | (val & mask));
}

static int aic26_sw_reset(const struct device *dev)
{
	int ret;

	ret = aic26_write(dev, AIC26_PAGE1, AIC26_P1_RESET,
			  AIC26_RESET_VALUE);
	if (ret == 0) {
		k_msleep(AIC26_RESET_WAIT_MS);
	}
	return ret;
}

static int aic26_setup_reference(const struct device *dev)
{
	return aic26_write(dev, AIC26_PAGE1, AIC26_P1_REFERENCE,
			   AIC26_VREFM_BIT | AIC26_IREFV_BIT);
}

static int aic26_calc_pll(uint32_t mclk, uint32_t fs, struct pll_cfg *out)
{
	static const uint32_t fsref_list[] = { 48000, 44100 };
	uint64_t num, k10k, vco_hz;
	uint32_t div_x2;
	uint32_t fsref;
	uint32_t mclk_p;
	uint32_t q, j, d;
	int8_t fs_div;

	memset(out, 0, sizeof(*out));

	if (fs == 0 || mclk == 0) {
		LOG_ERR("Invalid: MCLK=%u Fs=%u", mclk, fs);
		return -EINVAL;
	}

	for (int fi = 0; fi < ARRAY_SIZE(fsref_list); fi++) {
		fsref = fsref_list[fi];

		if ((fsref * 2) % fs != 0) {
			continue;
		}
		div_x2 = (fsref * 2) / fs;

		switch (div_x2) {
		case 2:
			fs_div = AIC26_FS_DIV_1;
			break;
		case 3:
			fs_div = AIC26_FS_DIV_1_5;
			break;
		case 4:
			fs_div = AIC26_FS_DIV_2;
			break;
		case 6:
			fs_div = AIC26_FS_DIV_3;
			break;
		case 8:
			fs_div = AIC26_FS_DIV_4;
			break;
		case 10:
			fs_div = AIC26_FS_DIV_5;
			break;
		case 11:
			fs_div = AIC26_FS_DIV_5_5;
			break;
		case 12:
			fs_div = AIC26_FS_DIV_6;
			break;
		default:
			continue;
		}

		out->fs_div = (uint8_t)fs_div;
		out->fsref_44k1 = (fi == 1);

		if ((mclk % (128U * fsref)) == 0) {
			q = mclk / (128U * fsref);

			if (q >= 2 && q <= 17) {
				out->pll_en = false;
				out->q = (uint8_t)q;
				LOG_DBG("No PLL: Q=%u fs_div=%u Fsref=%u",
					q, fs_div, fsref);
				return 0;
			}
		}

		/* PLL mode: Fsref = MCLK x K / (2048 x P), K = J.D */
		for (uint8_t p = 1; p <= 8; p++) {
			mclk_p = mclk / p;

			if (mclk_p < 2000000 || mclk_p > 20000000) {
				continue;
			}

			num = (uint64_t)fsref * 2048 * p * 10000ULL;
			k10k = num / mclk;
			j = (uint32_t)(k10k / 10000);
			d = (uint32_t)(k10k % 10000);

			if (d != 0) {
				if (j < 4 || j > 11 || mclk_p < 10000000) {
					continue;
				}
			} else {
				if (j < 4 || j > 55) {
					continue;
				}
			}

			if ((uint64_t)mclk * k10k != num) {
				continue;
			}

			/* Validate PLL VCO: 80 MHz <= MCLK*K/P <= 110 MHz */
			vco_hz = ((uint64_t)mclk * k10k) / (p *10000);

			if (vco_hz < 80000000ULL || vco_hz > 110000000ULL) {
				continue;
			}

			out->pll_en = true;
			out->p = p;
			out->j = (uint8_t)j;
			out->d = (uint16_t)d;
			LOG_DBG("PLL: P=%u J=%u D=%u fs_div=%u Fsref=%u",
				p, j, d, fs_div, fsref);
			return 0;
		}
	}

	LOG_ERR("No valid clock config for MCLK=%u Fs=%u", mclk, fs);
	return -EINVAL;
}

static int aic26_set_pll(const struct device *dev, const struct pll_cfg *pll)
{
	uint16_t pll1, pll2;
	int ret;
	uint8_t p_enc;
	uint8_t q_enc;

	if (pll->pll_en) {
		p_enc = (pll->p == 8) ? 0 : pll->p;

		pll1 = AIC26_PLLSEL_BIT |
		       ((p_enc << AIC26_PVAL_SHIFT) & AIC26_PVAL_MASK) |
		       ((pll->j << AIC26_JVAL_SHIFT) & AIC26_JVAL_MASK);
		pll2 = ((uint16_t)pll->d << AIC26_DVAL_SHIFT) &
		       AIC26_DVAL_MASK;
	} else {

		if (pll->q == 16) {
			q_enc = 0;
		} else if (pll->q == 17) {
			q_enc = 1;
		} else {
			q_enc = pll->q;
		}

		pll1 = (q_enc << AIC26_QVAL_SHIFT) & AIC26_QVAL_MASK;
		pll2 = 0;
	}

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_PLL2, pll2);
	if (ret < 0) {
		return ret;
	}

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_PLL1, pll1);
	if (ret < 0) {
		return ret;
	}

	if (pll->pll_en) {
		k_msleep(AIC26_PLL_LOCK_MS);
	}

	return 0;
}

static int aic26_adc_power_up(const struct device *dev)
{
	uint16_t reg;
	int ret;
	int polls;

	reg = (AIC26_AGCTG_M12 << AIC26_AGCTG_SHIFT) |
	      (AIC26_AGCTC_8_100 << AIC26_AGCTC_SHIFT) |
	      AIC26_AGCEN_BIT;

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_ADC_GAIN, reg);
	if (ret < 0) {
		return ret;
	}

	ret = aic26_update(dev, AIC26_PAGE2, AIC26_P2_AUDIO_CTL3,
			   AIC26_AGCNL_MASK,
			   AIC26_AGCNL_M60 << AIC26_AGCNL_SHIFT);
	if (ret < 0) {
		return ret;
	}

	ret = aic26_update(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL,
			   AIC26_PWDNC_BIT | AIC26_ADPWDN_BIT, 0);
	if (ret < 0) {
		return ret;
	}

	polls = AIC26_ADC_TIMEOUT_MS / AIC26_ADC_POLL_MS;

	for (int i = 0; i < polls; i++) {
		k_msleep(AIC26_ADC_POLL_MS);
		ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, &reg);
		if (ret < 0) {
			return ret;
		}
		if (!(reg & AIC26_ADPWDF_BIT)) {
			LOG_DBG("ADC ready in %d ms",
				(i + 1) * AIC26_ADC_POLL_MS);
			return 0;
		}
	}

	LOG_WRN("ADC power-up timeout (reg=0x%04x)", reg);
	return 0;
}

static int aic26_configure(const struct device *dev,
			   struct audio_codec_cfg *cfg)
{
	const struct tlv320aic26_config *dev_cfg = dev->config;
	struct tlv320aic26_data *data = dev->data;
	struct pll_cfg pll;
	uint16_t reg;
	uint8_t wlen;
	uint8_t datfm;
	bool codec_master;
	bool need_dac, need_adc;
	bool bclk_slave, fclk_slave;
	int ret;

	if (cfg->dai_type != AUDIO_DAI_TYPE_I2S) {
		LOG_ERR("Only I2S DAI supported");
		return -EINVAL;
	}

	switch (cfg->dai_route) {
	case AUDIO_ROUTE_PLAYBACK:
		need_dac = true;
		need_adc = false;
		break;
	case AUDIO_ROUTE_CAPTURE:
		need_dac = false;
		need_adc = true;
		break;
	case AUDIO_ROUTE_PLAYBACK_CAPTURE:
		need_dac = true;
		need_adc = true;
		break;
	default:
		LOG_ERR("Unsupported route %d", cfg->dai_route);
		return -EINVAL;
	}

	switch (cfg->dai_cfg.i2s.word_size) {
	case 16:
	    wlen = AIC26_WLEN_16;
	    break;
	case 20:
	    wlen = AIC26_WLEN_20;
	    break;
	case 24:
	    wlen = AIC26_WLEN_24;
	    break;
	case 32:
	    wlen = AIC26_WLEN_32;
	    break;
	default:
		LOG_ERR("Unsupported word size %u",
			cfg->dai_cfg.i2s.word_size);
		return -EINVAL;
	}

	switch (cfg->dai_cfg.i2s.format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		datfm = AIC26_DATFM_I2S;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		datfm = AIC26_DATFM_LJ;
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		datfm = AIC26_DATFM_RJ;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		datfm = AIC26_DATFM_DSP;
		break;
	default:
		LOG_ERR("Unsupported data format");
		return -EINVAL;
	}

	bclk_slave = (cfg->dai_cfg.i2s.options &
			   I2S_OPT_BIT_CLK_SLAVE) != 0;
	fclk_slave = (cfg->dai_cfg.i2s.options &
			   I2S_OPT_FRAME_CLK_SLAVE) != 0;

	if (bclk_slave != fclk_slave) {
		LOG_ERR("BCLK and LRCK master/slave must match");
		return -EINVAL;
	}

	codec_master = !bclk_slave;

	ret = aic26_calc_pll(dev_cfg->mclk_freq,
			     cfg->dai_cfg.i2s.frame_clk_freq, &pll);
	if (ret < 0) {
		return ret;
	}

	data->configured = false;

	ret = aic26_sw_reset(dev);
	if (ret < 0) {
		return ret;
	}

	ret = aic26_setup_reference(dev);
	if (ret < 0) {
		return ret;
	}

	ret = aic26_set_pll(dev, &pll);
	if (ret < 0) {
		return ret;
	}

	reg = ((uint16_t)wlen << AIC26_WLEN_SHIFT) |
	      ((uint16_t)datfm << AIC26_DATFM_SHIFT);

	if (need_dac) {
		reg |= ((uint16_t)pll.fs_div << AIC26_DACFS_SHIFT);
	}

	if (need_adc) {
		reg |= (AIC26_HPF_0045FS << AIC26_ADCHPF_SHIFT) |
		       (AIC26_ADCIN_MIC << AIC26_ADCIN_SHIFT) |
		       ((uint16_t)pll.fs_div << AIC26_ADCFS_SHIFT);
	}

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_AUDIO_CTL1, reg);
	if (ret < 0) {
		return ret;
	}

	reg = 0;

	if (pll.fsref_44k1) {
		reg |= AIC26_REFFS_BIT;
	}
	if (codec_master) {
		reg |= AIC26_SLVMS_BIT;
	}

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_AUDIO_CTL3, reg);
	if (ret < 0) {
		return ret;
	}

	if (need_adc) {
		ret = aic26_adc_power_up(dev);
		if (ret < 0) {
			return ret;
		}
		LOG_INF("ADC powered up with AGC");
	}

	data->configured  = true;
	data->route  = cfg->dai_route;

	LOG_INF("Configured: Fs=%u ws=%u %s",
		cfg->dai_cfg.i2s.frame_clk_freq, cfg->dai_cfg.i2s.word_size,
		codec_master ? "master" : "slave");

	return 0;
}

static void aic26_start_output(const struct device *dev)
{
	struct tlv320aic26_data *data = dev->data;
	uint16_t reg;
	int ret;
	bool ready = false;

	if (!data->configured) {
		LOG_ERR("Not configured");
		return;
	}

	if (data->route == AUDIO_ROUTE_CAPTURE) {
		LOG_ERR("Route is capture-only, cannot start output");
		return;
	}

	ret = aic26_update(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL,
			   AIC26_PWDNC_BIT | AIC26_DAPWDN_BIT |
			   AIC26_DAODRC_BIT | AIC26_ADPWDN_BIT,
			   AIC26_DAODRC_BIT | AIC26_ADPWDN_BIT);
	if (ret < 0) {
		return;
	}

	for (int i = 0; i < (AIC26_DAC_TIMEOUT_MS / AIC26_DAC_POLL_MS); i++) {
		k_msleep(AIC26_DAC_POLL_MS);
		ret = aic26_read(dev, AIC26_PAGE2,
				 AIC26_P2_POWER_CTL, &reg);
		if (ret < 0) {
			return;
		}
		if (!(reg & AIC26_DAPWDF_BIT)) {
			LOG_DBG("DAC ready in %d ms",
				(i + 1) * AIC26_DAC_POLL_MS);
			ready = true;
			break;
		}
	}

	if (!ready) {
		LOG_INF("DAC power-up timeout (reg=0x%04x)", reg);
	}

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_DAC_GAIN, 0x0000);
	if (ret < 0) {
		return;
	}

	LOG_INF("DAC output started");
}

static void aic26_stop_output(const struct device *dev)
{
	struct tlv320aic26_data *data = dev->data;
	int ret;

	if (!data->configured) {
		return;
	}

	ret = aic26_write(dev, AIC26_PAGE2, AIC26_P2_DAC_GAIN,
			  AIC26_DAC_MUTE_ALL);
	if (ret < 0) {
		return;
	}

	k_msleep(AIC26_DAC_SETTLE_MS);

	ret = aic26_update(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL,
			   AIC26_DAPWDN_BIT, AIC26_DAPWDN_BIT);
	if (ret < 0) {
		return;
	}

	LOG_INF("DAC output stopped");
}

static int aic26_set_property(const struct device *dev,
			      audio_property_t property,
			      audio_channel_t channel,
			      audio_property_value_t val)
{
	uint16_t reg;
	int ret;
	int db;
	uint8_t atten;
	uint8_t gain;

	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME: {
		db = CLAMP(val.vol, -63, 0);
		atten = (uint8_t)(-db * 2);

		if (atten > 0x7F) {
			atten = 0x7F;
		}

		ret = aic26_read(dev, AIC26_PAGE2,
				 AIC26_P2_DAC_GAIN, &reg);
		if (ret < 0) {
			return ret;
		}

		switch (channel) {
		case AUDIO_CHANNEL_FRONT_LEFT:
			reg = (reg & ~AIC26_DALVL_MASK) |
			      ((uint16_t)atten << AIC26_DALVL_SHIFT);
			break;
		case AUDIO_CHANNEL_FRONT_RIGHT:
			reg = (reg & ~AIC26_DARVL_MASK) | atten;
			break;
		case AUDIO_CHANNEL_ALL:
			reg = (reg & ~(AIC26_DALVL_MASK | AIC26_DARVL_MASK)) |
			      ((uint16_t)atten << AIC26_DALVL_SHIFT) | atten;
			break;
		default:
			return -EINVAL;
		}

		return aic26_write(dev, AIC26_PAGE2,
				   AIC26_P2_DAC_GAIN, reg);
	}

	case AUDIO_PROPERTY_OUTPUT_MUTE: {
		ret = aic26_read(dev, AIC26_PAGE2,
				 AIC26_P2_DAC_GAIN, &reg);
		if (ret < 0) {
			return ret;
		}

		switch (channel) {
		case AUDIO_CHANNEL_FRONT_LEFT:
			if (val.mute) {
				reg |= AIC26_DALMU_BIT;
			} else {
				reg &= ~AIC26_DALMU_BIT;
			}
			break;
		case AUDIO_CHANNEL_FRONT_RIGHT:
			if (val.mute) {
				reg |= AIC26_DARMU_BIT;
			} else {
				reg &= ~AIC26_DARMU_BIT;
			}
			break;
		case AUDIO_CHANNEL_ALL:
			if (val.mute) {
				reg |= AIC26_DALMU_BIT | AIC26_DARMU_BIT;
			} else {
				reg &= ~(AIC26_DALMU_BIT | AIC26_DARMU_BIT);
			}
			break;
		default:
			return -EINVAL;
		}

		return aic26_write(dev, AIC26_PAGE2,
				   AIC26_P2_DAC_GAIN, reg);
	}

	case AUDIO_PROPERTY_INPUT_VOLUME: {
		db = CLAMP(val.vol, 0, 59);
		gain = (uint8_t)(db * 2);

		if (gain > 0x77) {
			gain = 0x77;
		}

		return aic26_update(dev, AIC26_PAGE2, AIC26_P2_ADC_GAIN,
				    AIC26_ADMUT_BIT | AIC26_ADPGA_MASK,
				    (uint16_t)gain << AIC26_ADPGA_SHIFT);
	}

	case AUDIO_PROPERTY_INPUT_MUTE:
		return aic26_update(dev, AIC26_PAGE2, AIC26_P2_ADC_GAIN,
				    AIC26_ADMUT_BIT,
				    val.mute ? AIC26_ADMUT_BIT : 0);

	default:
		return -ENOTSUP;
	}
}
static int aic26_apply_properties(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int aic26_init(const struct device *dev)
{
	const struct tlv320aic26_config *cfg = dev->config;
	struct tlv320aic26_data *data = dev->data;
	uint16_t reg;
	int ret;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	ret = aic26_sw_reset(dev);
	if (ret < 0) {
		LOG_ERR("Reset failed: %d", ret);
		return ret;
	}

	ret = aic26_read(dev, AIC26_PAGE2, AIC26_P2_POWER_CTL, &reg);
	if (ret < 0) {
		LOG_ERR("Cannot communicate with codec: %d", ret);
		return ret;
	}

	if (reg == 0x0000 || reg == 0xFFFF) {
		LOG_ERR("SPI read returned 0x%04X - check wiring/mode", reg);
		return -EIO;
	}

	LOG_INF("Power CTL after reset: 0x%04X", reg);

	ret = aic26_setup_reference(dev);
	if (ret < 0) {
		return ret;
	}

	data->configured = false;

	LOG_INF("TLV320AIC26 ready (MCLK=%u Hz)", cfg->mclk_freq);
	return 0;
}

static const struct audio_codec_api aic26_api = {
	.configure        = aic26_configure,
	.start_output     = aic26_start_output,
	.stop_output      = aic26_stop_output,
	.set_property     = aic26_set_property,
	.apply_properties = aic26_apply_properties,
};

#define TLV320AIC26_INIT(n)                                             \
	static struct tlv320aic26_data aic26_data_##n;                  \
                                                                        \
	static const struct tlv320aic26_config aic26_cfg_##n = {        \
		.spi = SPI_DT_SPEC_INST_GET(n,                          \
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |         \
			SPI_MODE_CPHA | SPI_WORD_SET(8),                \
			0),                                             \
		.mclk_freq = DT_INST_PROP(n, mclk_frequency),          \
	};                                                              \
                                                                        \
	DEVICE_DT_INST_DEFINE(n,                                        \
		aic26_init, NULL,                                       \
		&aic26_data_##n, &aic26_cfg_##n,                        \
		POST_KERNEL,                                            \
		CONFIG_AUDIO_CODEC_INIT_PRIORITY,                       \
		&aic26_api);

DT_INST_FOREACH_STATUS_OKAY(TLV320AIC26_INIT)
