/*
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Charger APIs
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_HY4245_H_
#define ZEPHYR_INCLUDE_DRIVERS_HY4245_H_

/**
 * @brief Charger Interface
 * @defgroup charger_interface Charger Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/device.h>
#include <zephyr/drivers/fuel_gauge.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief Set the HY4245 sensor into calibration mode.
 *
 * This function puts the HY4245 sensor into calibration mode, allowing
 * calibration data to be accessed and modified.
 *
 * @param dev Pointer to the device structure for the sensor.
 *
 * @return 0 if successful, error code otherwise.
 */
int hy4245_set_calibration_mode(const struct device *dev);

/**
 * @brief Enable flash access for the HY4245 sensor.
 *
 * This function enables access to the flash memory of the HY4245 sensor,
 * which is required before performing any flash memory operations.
 *
 * @param dev Pointer to the device structure for the sensor.
 *
 * @return 0 if successful, error code otherwise.
 */
int hy4245_enable_flash_access(const struct device *dev);

/**
 * @brief Access flash data of the HY4245 sensor.
 *
 * This function allows reading from or writing to the flash memory of the
 * HY4245 sensor. It is used to access calibration data or other parameters
 * stored in the flash memory.
 *
 * @param dev Pointer to the device structure for the sensor.
 * @param subclass Subclass of the data to access.
 * @param block Block number within the subclass.
 * @param data Buffer for the data to read or write.
 * @param count Number of bytes to read or write.
 * @param is_read Flag indicating whether to read (true) or write (false) data.
 *
 * @return 0 if successful, error code otherwise.
 */
int hy4245_access_flash_data(const struct device *dev, uint8_t subclass, uint8_t block,
			     uint8_t *data, uint8_t count, bool is_read);

/**
 * @brief Resets the HY4245 device.
 *
 * This function performs a reset operation on the HY4245 device specified by the
 * @a dev parameter. The reset process reinitializes the device to its default state,
 * clearing any existing configurations or data. It is typically called when the device
 * needs to be brought back to a known good state or when recovering from an error condition.
 *
 * @param dev Pointer to the device structure of the HY4245 device to be reset.
 * @return int Returns 0 if the reset operation was successful, or a negative error code if it
 * failed.
 */
int hy4245_reset(const struct device *dev);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_INCLUDE_DRIVERS_HY4245_H_ */
