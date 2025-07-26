/*
 * Copyright (c) 2025 Texas Instruments
 * Copyright (c) 2025 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DAC_MSPM0_WRAPPER_H_
#define ZEPHYR_INCLUDE_DRIVERS_DAC_MSPM0_WRAPPER_H_

#include <zephyr/drivers/dac.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set up the DAC channel with the specified configuration.
 *
 * This function initializes the MSPM0 G-Series DAC with the given
 * configuration. It configures resolution, enables internal routing
 * and output buffering, and performs self-calibration.
 *
 * @param channel_cfg Pointer to the DAC channel configuration structure.
 *
 * @retval 0        DAC channel successfully configured.
 * @retval -EINVAL  Invalid channel ID (only channel 0 is supported).
 * @retval -ENOTSUP Unsupported resolution (only 8 or 12 bits are allowed).
 * @retval -EAGAIN  DAC enable failed after configuration.
 *
 * @note Only single-channel DAC operation is currently supported.
 * @note Only resolutions of 8 or 12 bits are supported.
 * @note To observe DAC output, both `buffered` and `internal` must be enabled.
 */
int dac_mspm0_channel_setup(const struct dac_channel_cfg *cfg);

/**
 * @brief Write a value to the specified DAC channel.
 *
 * Writes a digital value to the DAC after verifying that the channel has been
 * configured and the value is within the valid range for the configured
 * resolution and representation format (binary or two's complement).
 *
 * @param channel   DAC channel ID to write to (only channel 0 is supported).
 * @param value     Digital value to write to the DAC.
 *
 * @retval 0        Value successfully written to DAC output.
 * @retval -EIO     DAC channel has not been initialized (setup not done).
 * @retval -EINVAL  Invalid channel ID or value out of supported range.
 */
int dac_mspm0_write_value(uint8_t channel, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_DAC_MSPM0_WRAPPER_H_ */
