/*
 * copyright (c) 2025 Linumiz GmbH
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#define DT_DRV_COMPAT infineon_ifx_sar

#include <errno.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sar_infineon_cat1, CONFIG_ADC_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#include <cy_sar2.h>
#include <cy_sysclk.h>

#define SAR_RESOLUTION 	12
#define GROUP_END 	BIT(11)
#define SAR_MIN_FREQ_HZ    2000000
#define SAR_MAX_FREQ_HZ 26670000
#define SAMPLING_DIV_FREQ_HZ 1000000
#define MIN_SAMP_CLK	1
#define MAX_SAMP_CLK	4095
#define PWR_DELAY_MS	1

struct ifx_cat1_sar_config {
	PASS_SAR_Type *base;
	const struct pinctrl_dev_config *pinctrl;
	uint32_t int_number;
	uint32_t priority;
	uint32_t frequency;
	uint32_t clock_peri_group;
        uint32_t clock_id;
        uint8_t peri_div_type;
        uint8_t peri_div_type_inst;
};

struct ifx_cat1_sar_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
};

static void ifx_cat1_sar_isr(const struct device *dev)
{
	const struct ifx_cat1_sar_config *config = dev->config;
        struct ifx_cat1_sar_data *data = dev->data;
	uint32_t channels = data->ctx.sequence.channels;
	uint32_t last_channel = find_msb_set(channels)-1;
	uint32_t ch = 0;
	uint32_t status = Cy_SAR2_Channel_GetInterruptStatus(config->base, last_channel);
	
	LOG_INF("ISR entered! last_ch=%d, status=0x%08x", last_channel, status);
	
	Cy_SAR2_Channel_ClearInterrupt(config->base, last_channel, CY_SAR2_INT_GRP_DONE);
	
	if (status & CY_SAR2_INT_GRP_DONE) {
		LOG_INF("GRP_DONE interrupt - reading channels");
		while (channels != 0)
		{
			ch = find_lsb_set(channels)-1;
			*data->buffer++ = Cy_SAR2_Channel_GetResult(config->base, ch, NULL);
			LOG_INF("Read ch%d result: %d", ch, *(data->buffer-1));
			channels &= ~BIT(ch);
		}
		data->buffer--;
        	(config->base)->CH[last_channel].TR_CTL &= ~GROUP_END;
	} else {
		LOG_WRN("ISR fired but no GRP_DONE flag!");
	}

	LOG_INF("Calling adc_context_on_sampling_done");
	adc_context_on_sampling_done(&data->ctx, dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat)
{
	struct ifx_cat1_sar_data *data = CONTAINER_OF(ctx, struct ifx_cat1_sar_data, ctx);

	if (repeat) {
		data->buffer = data->repeat_buffer;
	} else {
		data->buffer++;
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ifx_cat1_sar_data *data = CONTAINER_OF(ctx, struct ifx_cat1_sar_data, ctx);
	const struct device *dev = data->dev;
	const struct ifx_cat1_sar_config *config = dev->config;
	uint8_t last_ch = find_msb_set(ctx->sequence.channels)-1;
	uint8_t first_ch = find_lsb_set(ctx->sequence.channels)-1;
        
	LOG_INF("Starting sampling: first_ch=%d, last_ch=%d, channels=0x%x", 
	        first_ch, last_ch, ctx->sequence.channels);
	
	(config->base)->CH[last_ch].TR_CTL |= GROUP_END;

	Cy_SAR2_Channel_ClearInterrupt(config->base, last_ch, CY_SAR2_INT_GRP_DONE);
	Cy_SAR2_Channel_SetInterruptMask(config->base, last_ch, CY_SAR2_INT_GRP_DONE);
	
	LOG_INF("Triggering channel %d...", first_ch);
	Cy_SAR2_Channel_SoftwareTrigger(config->base, first_ch);
	
	uint32_t tr_pend = (config->base)->TR_PEND;
	uint32_t grp_stat = (config->base)->CH[first_ch].GRP_STAT;
	uint32_t busy = (grp_stat >> 16) & 1;
	LOG_INF("After trigger: TR_PEND=0x%08x, GRP_STAT=0x%08x, BUSY=%d", 
	        tr_pend, grp_stat, busy);
}

static int ifx_cat1_sar_channel_setup(const struct device *dev,
                                      const struct adc_channel_cfg *channel_cfg)
{
	cy_stc_sar2_channel_config_t cy_ch_cfg;
	cy_en_sar2_status_t status;
	const struct ifx_cat1_sar_config *config = dev->config;
	struct ifx_cat1_sar_data *data = dev->data;
	int ret = 0;
	uint16_t sample_time = 0;
	uint8_t channel_id = channel_cfg->channel_id;
	
	LOG_INF("Setting up ADC channel %d: pin=%d, sample_time=%d", 
	        channel_id, channel_cfg->input_positive, sample_time);

	if (channel_id > CY_SAR2_CHAN_NUM(config->base)) {
                return -EINVAL;
        }

	if (channel_cfg->gain != ADC_GAIN_1) {
                return -ENOTSUP;
        }

        if (channel_cfg->differential) {
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_VDD_1) {
		return -ENOTSUP;
	}


        cy_ch_cfg.channelHwEnable = true;
        cy_ch_cfg.triggerSelection = CY_SAR2_TRIGGER_OFF;
        cy_ch_cfg.channelPriority = 0U;
        cy_ch_cfg.preenptionType = CY_SAR2_PREEMPTION_FINISH_RESUME;
        cy_ch_cfg.isGroupEnd = false;
        cy_ch_cfg.doneLevel = CY_SAR2_DONE_LEVEL_LEVEL;
	cy_ch_cfg.pinAddress = channel_cfg->input_positive;
	cy_ch_cfg.portAddress = CY_SAR2_PORT_ADDRESS_SARMUX0;
        cy_ch_cfg.extMuxSelect = 0U;
        cy_ch_cfg.extMuxEnable = false;
        cy_ch_cfg.preconditionMode = CY_SAR2_PRECONDITION_MODE_OFF;
        cy_ch_cfg.overlapDiagMode = CY_SAR2_OVERLAP_DIAG_MODE_OFF;
	
  	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		switch (ADC_ACQ_TIME_UNIT(channel_cfg->acquisition_time)) {
		case ADC_ACQ_TIME_TICKS:
			sample_time = ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time);
			break;
		case ADC_ACQ_TIME_MICROSECONDS:
			sample_time = ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time) *
				     ((config->frequency) / SAMPLING_DIV_FREQ_HZ);
			break;
		case ADC_ACQ_TIME_NANOSECONDS:
			sample_time = (ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time) / 1000) *
				      ((config->frequency) / SAMPLING_DIV_FREQ_HZ);
			break;
		default:
			LOG_ERR("Selected ADC acquisition time units is not valid");
			return -EINVAL;
		}
	}
	sample_time = CLAMP(sample_time, MIN_SAMP_CLK, MAX_SAMP_CLK);
	cy_ch_cfg.sampleTime = sample_time;
        cy_ch_cfg.calibrationValueSelect = CY_SAR2_CALIBRATION_VALUE_REGULAR;
        cy_ch_cfg.resultAlignment = CY_SAR2_RESULT_ALIGNMENT_RIGHT;
        cy_ch_cfg.signExtention = CY_SAR2_SIGN_EXTENTION_UNSIGNED;
        cy_ch_cfg.postProcessingMode = CY_SAR2_POST_PROCESSING_MODE_NONE;
        cy_ch_cfg.averageCount = 1U;
        cy_ch_cfg.rightShift = 0U;
        cy_ch_cfg.positiveReload = 0U;
        cy_ch_cfg.negativeReload = 0U;
        cy_ch_cfg.rangeDetectionMode = CY_SAR2_RANGE_DETECTION_MODE_BELOW_LO;
        cy_ch_cfg.rangeDetectionLoThreshold = 0U;
        cy_ch_cfg.rangeDetectionHiThreshold = 0xFFFU;
        cy_ch_cfg.interruptMask = 0U;

	adc_context_lock(&data->ctx, false, NULL);
	
	status = Cy_SAR2_Channel_Init(config->base, channel_id, &cy_ch_cfg);
        if (status != CY_SAR2_SUCCESS) {
                LOG_ERR("Channel init failed: status=%d", status);
                ret = -EIO;
		goto unlock;
        }
	
	LOG_INF("Channel %d initialized, enabling interrupt %d", 
	        channel_id, config->int_number + channel_id);

        enable_sys_int(config->int_number + channel_id, config->priority,
		    (void (*)(const void *))(void *)ifx_cat1_sar_isr, dev);

unlock:
	adc_context_release(&data->ctx, 0);
	return ret;
}

static int ifx_cat1_sar_read_internal(const struct device *dev,
				      const struct adc_sequence *sequence)
{
	struct ifx_cat1_sar_data *data = dev->data;
	size_t exp_size;
	uint8_t count = __builtin_popcount(sequence->channels);
	
	LOG_INF("ADC read started: channels=0x%x, count=%d", sequence->channels, count);

        if (sequence->resolution != SAR_RESOLUTION) {
                LOG_ERR("Unsupported resolution %u (only %u-bit supported)",
                        sequence->resolution, SAR_RESOLUTION);
                return -EINVAL;
        }

        if (sequence->oversampling != 0) {
                LOG_ERR("Oversampling not supported");
                return -ENOTSUP;
        }
	
	if (count == 0) {
                LOG_ERR("No channels selected");
                return -EINVAL;
        }

	exp_size = count * sizeof(uint16_t);
	if (sequence->options) {
                exp_size *= (1 + sequence->options->extra_samplings);
        }

	if (sequence->buffer_size < exp_size) {
                LOG_ERR("Buffer too small: need %zu bytes, have %zu",
                        exp_size, sequence->buffer_size);
                return -ENOMEM;
        }

	data->buffer = sequence->buffer;
	data->repeat_buffer = data->buffer;
	
	LOG_INF("Starting ADC read context...");
	adc_context_start_read(&data->ctx, sequence);
	
	LOG_INF("Waiting for ADC completion...");
	int result = adc_context_wait_for_completion(&data->ctx);
	LOG_INF("ADC read completed with result: %d", result);
	return result;
}

static int ifx_cat1_sar_read(const struct device *dev,
                             const struct adc_sequence *sequence)
{
	struct ifx_cat1_sar_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, false, NULL);
	ret = ifx_cat1_sar_read_internal(dev, sequence);
	adc_context_release(&data->ctx, ret);

	return ret;
}

#ifdef CONFIG_ADC_ASYNC
static int ifx_cat1_sar_read_async(const struct device *dev,
                                   const struct adc_sequence *sequence,
                                   struct k_poll_signal *async)
{
	struct ifx_cat1_sar_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, true, async);
	ret = ifx_cat1_sar_read_internal(dev, sequence);
	adc_context_release(&data->ctx, ret);
	
	return ret;
}
#endif

static int ifx_cat1_sar_hw_init(const struct device *dev)
{
	const struct ifx_cat1_sar_config *config = dev->config;
        cy_stc_sar2_config_t sar_cfg = {0};
        cy_en_sar2_status_t status;
	sar_cfg.preconditionTime = 0U;
        sar_cfg.powerupTime = PWR_DELAY_MS * (config->frequency / SAMPLING_DIV_FREQ_HZ);
        sar_cfg.enableIdlePowerDown = true;
        sar_cfg.msbStretchMode = CY_SAR2_MSB_STRETCH_MODE_2CYCLE;
        sar_cfg.enableHalfLsbConv = true;
        sar_cfg.sarMuxEnable = true;
        sar_cfg.adcEnable = true;
        sar_cfg.sarIpEnable = true;

	status = Cy_SAR2_Init(config->base, &sar_cfg);
        if (status != CY_SAR2_SUCCESS) {
                LOG_ERR("SAR2 Init failed: status=%d", status);
                return -EIO;
        }
	
	uint32_t ctl = (config->base)->CTL;
	uint32_t enabled = (ctl >> 31) & 1;
	uint32_t adc_en = (ctl >> 28) & 1;
	uint32_t mux_en = (ctl >> 27) & 1;
	LOG_INF("SAR CTL register: 0x%08x (ENABLED=%d, ADC_EN=%d, SARMUX_EN=%d)", 
	        ctl, enabled, adc_en, mux_en);

	return 0;
}

static int ifx_clock_config(const struct device *dev, uint32_t target_freq)
{
    const struct ifx_cat1_sar_config *config = dev->config;
    uint32_t divider;
    uint32_t hf_clock_frequency;
    
    if (target_freq < SAR_MIN_FREQ_HZ || target_freq > SAR_MAX_FREQ_HZ) {
        LOG_ERR("Invalid SAR frequency: %u", target_freq);
        return -EINVAL;
    }

    clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(clk_hf2)),
                                 NULL, &hf_clock_frequency);
    

    divider = (hf_clock_frequency + (target_freq / 2)) / target_freq;
    
    Cy_SysClk_PeriPclkDisableDivider(config->clock_peri_group,
                                     config->peri_div_type,
                                     config->peri_div_type_inst);

    Cy_SysClk_PeriPclkSetDivider(config->clock_peri_group,
                                 config->peri_div_type,
                                 config->peri_div_type_inst,
                                 divider);
    Cy_SysClk_PeriPclkEnableDivider(config->clock_peri_group,
                                    config->peri_div_type,
                                    config->peri_div_type_inst);
    Cy_SysClk_PeriPclkAssignDivider(config->clock_id,
                                   config->peri_div_type,
                                   config->peri_div_type_inst);
    return 0;
}

static int ifx_cat1_sar_init(const struct device *dev) {
	
	const struct ifx_cat1_sar_config *config = dev->config;
	struct ifx_cat1_sar_data *data = dev->data;
	int ret = 0;

	LOG_INF("Initilizing infineon CAT1 SAR2 ADC");
	
	data->dev = dev;

	ret = ifx_clock_config(dev, config->frequency);
	if (ret < 0) {
		return ret;
	}

	ret = ifx_cat1_sar_hw_init(dev);
	if(ret < 0) {
		return ret;
	}

	LOG_INF("ADC Intilized successfully");	
	adc_context_unlock_unconditionally(&data->ctx);
	return ret;
}

static DEVICE_API(adc, ifx_cat1_driver_api) = {
	.channel_setup = ifx_cat1_sar_channel_setup,
	.read	       = ifx_cat1_sar_read,
#ifdef CONFIG_ADC_SYNC
	.read_async    = ifx_cat1_sar_read_async,
#endif
};

#define IFX_SAR_ADC_INIT(n)								       \
	static struct ifx_cat1_sar_data ifx_cat1_sar_data_##n = {                              \
		ADC_CONTEXT_INIT_TIMER(ifx_cat1_sar_data_##n, ctx),                            \
		ADC_CONTEXT_INIT_LOCK(ifx_cat1_sar_data_##n, ctx),                             \
		ADC_CONTEXT_INIT_SYNC(ifx_cat1_sar_data_##n, ctx),                             \
	};											       \
	static const struct ifx_cat1_sar_config ifx_cat1_sar_cfg_##n = {                       \
                .base = (PASS_SAR_Type *)DT_INST_REG_ADDR(n),                                  \
		.clock_peri_group = DT_INST_PROP(n, ifx_peri_group),                           \
		.int_number = DT_INST_PROP_BY_IDX(n, system_interrupts, 0),                    \
		.priority = DT_INST_PROP_BY_IDX(n, system_interrupts, 1),                      \
		.frequency = DT_INST_PROP_OR(n, clock_frequency, SAR_MAX_FREQ_HZ),	       \
                .clock_id = DT_INST_PROP(n, ifx_peri_clk),                                     \
                .peri_div_type = DT_INST_PROP(n, ifx_peri_div),                                \
                .peri_div_type_inst = DT_INST_PROP(n, ifx_peri_div_inst),                      \
        };                                                                                     \
	DEVICE_DT_INST_DEFINE(n, &ifx_cat1_sar_init, NULL, &ifx_cat1_sar_data_##n,             \
			      &ifx_cat1_sar_cfg_##n, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,    \
			      &ifx_cat1_driver_api);                                           \

DT_INST_FOREACH_STATUS_OKAY(IFX_SAR_ADC_INIT)
