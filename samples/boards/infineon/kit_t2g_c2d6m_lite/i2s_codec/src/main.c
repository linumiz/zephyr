/*
 * Copyright (c) 2026 Linumiz
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <zephyr/toolchain.h>

#include "song.h"   /* provides __16kHz_16bit_stereo_sine_pcm + _len */

#define I2S_CODEC_TX       DT_ALIAS(i2s_codec_tx)
#define SAMPLE_FREQUENCY   CONFIG_SAMPLE_FREQ
#define SAMPLE_BIT_WIDTH   16U
#define NUMBER_OF_CHANNELS 2U

/* Keep this small (your i2s_infineon driver was failing with big blocks) */
#define BLOCK_SIZE         168U

#define TIMEOUT_MS         2000U

/* Provide enough slab blocks so the driver queue can stay fed */
#define BLOCK_COUNT        16

K_MEM_SLAB_DEFINE_IN_SECT_STATIC(mem_slab, __nocache, BLOCK_SIZE, BLOCK_COUNT, 4);

static int i2s_send_block_retry(const struct device *i2s_dev, void *mem_block, size_t block_size)
{
    int ret;

    do {
        ret = i2s_buf_write(i2s_dev, mem_block, block_size);
        if (ret == -EAGAIN) {
            /* TX queue full -> let ISR/DMA consume and retry */
            k_msleep(1);
        }
    } while (ret == -EAGAIN);

    return ret;
}

int main(void)
{
    const struct device *const i2s_dev  = DEVICE_DT_GET(I2S_CODEC_TX);

    struct i2s_config i2s_cfg = {0};
    int ret;
#if 1
    const struct device *const codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));
    struct audio_codec_cfg audio_cfg = {0};
    const struct device *const i2c_dev  = DEVICE_DT_GET(DT_NODELABEL(scb11));
    if (!device_is_ready(i2c_dev)) {
        printk("ERROR: I2C device not ready!\n");
        return -1;
    }

    ret = i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER);
    if (ret < 0) {
        printk("ERROR: Failed to configure I2C (err %d)\n", ret);
        return -1;
    }

    if (!device_is_ready(i2s_dev)) {
        printk("ERROR: %s not ready\n", i2s_dev->name);
        return -1;
    }
    if (!device_is_ready(codec_dev)) {
        printk("ERROR: codec not ready\n");
        return -1;
    }

    /* Codec setup (playback) */
    audio_cfg.dai_route = AUDIO_ROUTE_PLAYBACK;
    audio_cfg.dai_type  = AUDIO_DAI_TYPE_I2S;
    audio_cfg.dai_cfg.i2s.word_size      = SAMPLE_BIT_WIDTH;
    audio_cfg.dai_cfg.i2s.channels       = NUMBER_OF_CHANNELS;
    audio_cfg.dai_cfg.i2s.format         = I2S_FMT_DATA_FORMAT_I2S;

#ifdef CONFIG_USE_CODEC_CLOCK
    audio_cfg.dai_cfg.i2s.options        = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
#else
    audio_cfg.dai_cfg.i2s.options        = I2S_OPT_FRAME_CLK_SLAVE  | I2S_OPT_BIT_CLK_SLAVE;
#endif

    audio_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_FREQUENCY;
    audio_cfg.dai_cfg.i2s.mem_slab       = &mem_slab;
    audio_cfg.dai_cfg.i2s.block_size     = BLOCK_SIZE;

   audio_codec_configure(codec_dev, &audio_cfg);
   audio_codec_start_output(codec_dev);

  // k_msleep(200);
#endif

    /* I2S TX setup */
    i2s_cfg.word_size      = SAMPLE_BIT_WIDTH;
    i2s_cfg.channels       = NUMBER_OF_CHANNELS;
    i2s_cfg.format         = I2S_FMT_DATA_FORMAT_I2S;

#ifdef CONFIG_USE_CODEC_CLOCK
    /* Codec provides clocks, so MCU is slave */
    i2s_cfg.options        = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE;
#else
    /* MCU provides clocks */
    i2s_cfg.options        = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
#endif

    i2s_cfg.frame_clk_freq = SAMPLE_FREQUENCY;
    i2s_cfg.mem_slab       = &mem_slab;
    i2s_cfg.block_size     = BLOCK_SIZE;
    i2s_cfg.timeout        = TIMEOUT_MS;

    ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
    if (ret < 0) {
        printk("ERROR: i2s_configure(TX) failed: %d\n", ret);
        return ret;
    }

    /* Prime the TX queue with a few blocks before START */
    size_t off = 0;
    const uint8_t *buf = __16kHz16bit_stereo_sine_pcm;
    const size_t len   = __16kHz16bit_stereo_sine_pcm_len;
    printf("length = %d \n", len);

    BUILD_ASSERT(BLOCK_SIZE <= __16kHz16bit_stereo_sine_pcm_len,
                 "BLOCK_SIZE bigger than sine buffer."); /* [file:16] pattern */

    for (int i = 0; i < 4; i++) {
        if (off + BLOCK_SIZE > len) {
            off = 0;
        }

        ret = i2s_send_block_retry(i2s_dev, (void *)&buf[off], BLOCK_SIZE);
        if (ret < 0) {
            printk("ERROR: prime i2s_buf_write failed: %d\n", ret);
            return ret;
        }

        off += BLOCK_SIZE;
    }

    ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret < 0) {
        printk("ERROR: I2S_TRIGGER_START failed: %d\n", ret);
        return ret;
    }

    printk("Sine playback started (%u Hz, %u-bit, %u ch)\n",
           SAMPLE_FREQUENCY, SAMPLE_BIT_WIDTH, NUMBER_OF_CHANNELS);
    /* Continuous streaming loop */
    while (1) {
        if (off + BLOCK_SIZE > len) {
            off = 0;
        }
        ret = i2s_send_block_retry(i2s_dev, (void *)&buf[off], BLOCK_SIZE);
        if (ret < 0) {
            printk("ERROR: i2s_buf_write failed: %d\n", ret);
            break;
        }
        off += BLOCK_SIZE;
    }
    /* If you ever exit, stop TX cleanly */
    (void)i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_STOP);
    (void)i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
    return 0;
}
