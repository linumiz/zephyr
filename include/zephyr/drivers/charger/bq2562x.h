/*
 * Copyright 2025 Testo SE & Co. KGaA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Charger APIs
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_BQ2562X_H_
#define ZEPHYR_INCLUDE_DRIVERS_BQ2562X_H_

/**
 * @brief Charger Interface
 * @defgroup charger_interface Charger Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/device.h>
#include <zephyr/drivers/charger.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief Charger current protection threshold
 */
enum charger_current_threshold {
	/*setting charger current protection threshold 1.5 A*/
	CHARGER_CURRENT_THRESHOLD_1_5_A = 0,
	/*setting charger current protection threshold 3 A*/
	CHARGER_CURRENT_THRESHOLD_3_A = 1,
	/*setting charger current protection threshold 6 A*/
	CHARGER_CURRENT_THRESHOLD_6_A = 2,
	/*setting charger current protection threshold 12 A*/
	CHARGER_CURRENT_THRESHOLD_12_A = 3,
};

/**
 * @brief The charge rate definition for the fast charge stage
 */
enum charger_rate {
	/*setting charge rate 1 C*/
	CHARGER_RATE_1_C = 0,
	/*setting charge rate 2 C*/
	CHARGER_RATE_2_C = 1,
	/*setting charge rate 4 C*/
	CHARGER_RATE_4_C = 2,
	/*setting charge rate 6 C*/
	CHARGER_RATE_6_C = 3,
};

/**
 * @brief Battery Recharge Threshold Offset
 */
enum charger_recharge_threshold_offset {
	/*setting recharge threshold offset 100 mV*/
	CHARGER_VREG_100_MV = 0,
	/*setting recharge threshold offset 200 mV*/
	CHARGER_VREG_200_MV = 1,
};

/**
 * @brief Whether to turn on the ADC sampling function of the charging chip
 */
enum charger_battery_adc_sampling {
	/* turn off the ADC sampling function of the charging chip*/
	CHARGER_ADC_DISABLE = 0,
	/* turn on the ADC sampling function of the charging chip*/
	CHARGER_ADC_ENABLE = 1,
};

/**
 * @brief Force D+/D- detection
 */
enum charger_dpdm_detection {
	/* Do not force DPDM detection*/
	CHARGER_INDET_DISABLE = 0,
	/* Force DPDM algorithm detection*/
	CHARGER_INDET_ENABLE = 1,
};

/**
 * @brief Fast charge, trickle charge and pre-charge timer status
 */
enum charger_timer_state {
	/*charge timer is normal*/
	CHARGER_TMR_STATE_NORMAL = 0,
	/*charge timer is expired*/
	CHARGER_TMR_STATE_TIMER_EXPIRED = 1,
};

/**
 * @brief Set the charge current threshold for the bq2562x charger.
 *
 * This function configures the charge current threshold on the bq2562x charger
 * device. The charge current threshold determines the point at which the charger
 * switches between different charging states.
 *
 * @param dev Pointer to the device structure representing the bq2562x charger.
 * @param current_threshold The desired charge current threshold to set.
 *
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_set_charge_current_threshold(const struct device *dev,
					 enum charger_current_threshold current_threshold);

/**
 * @brief Set the charge rate for the bq2562x charger.
 *
 * This function sets the charge rate on the bq2562x charger device. The charge
 * rate determines the speed at which the battery is charged.
 *
 * @param dev Pointer to the device structure representing the bq2562x charger.
 * @param charge_rate The desired charge rate to set.
 *
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_set_charge_rate(const struct device *dev, enum charger_rate charge_rate);

/**
 * @brief Set the battery recharge threshold offset for the bq2562x charger.
 *
 * This function configures the recharge threshold offset on the bq2562x charger
 * device. The recharge threshold determines the voltage level at which the
 * charger considers the battery to be in need of recharging.
 *
 * @param dev Pointer to the device structure representing the bq2562x charger.
 * @param offset The desired recharge threshold offset to set.
 *
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_set_battery_recharge_threshold_offset(const struct device *dev,
						  enum charger_recharge_threshold_offset offset);

/**
 * @brief Enable or disable ADC sampling for the bq2562x charger.
 *
 * This function enables or disables the ADC (Analog-to-Digital Converter)
 * sampling feature on the bq2562x charger device. ADC sampling is used to
 * measure various electrical parameters.
 *
 * @param dev Pointer to the device structure representing the bq2562x charger.
 * @param enable Flag to enable or disable ADC sampling.
 *
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_set_adc_sampling(const struct device *dev, enum charger_battery_adc_sampling enable);

/**
 * @brief Enable or disable DPDM detection for the bq2562x charger.
 *
 * This function enables or disables the DPDM (Data Pair Positive and Data Pair
 * Negative) detection feature on the bq2562x charger device. DPDM detection is
 * used for USB charging port detection.
 *
 * @param dev Pointer to the device structure representing the bq2562x charger.
 * @param enable Flag to enable or disable DPDM detection.
 *
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_set_dpdm_detection(const struct device *dev, enum charger_dpdm_detection enable);

/**
 * @brief Get the timer status of the bq2562x charger.
 *
 * This function retrieves the current state of a specific timer on the bq2562x
 * charger device. Timers are used to manage various charging and protection
 * operations.
 *
 * @param dev Pointer to the device structure representing the bq2562x charger.
 * @param state Pointer to store the retrieved timer state.
 *
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_timer_status(const struct device *dev, enum charger_timer_state *state);

/**
 * @brief Get the TDIE ADC value for the bq2562x charger.
 *
 * This function retrieves the ADC (Analog-to-Digital Converter) value
 * corresponding to the temperature of the bq2562x charger's die. This is used
 * for temperature monitoring and protection.
 *
 * @param dev Pointer to the device structure representing the bq2562x charger.
 * @param temperature Pointer to store the retrieved temperature value.
 *
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_tdie_adc(const struct device *dev, int32_t *temperature);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_INCLUDE_DRIVERS_BQ2562X_H_ */
