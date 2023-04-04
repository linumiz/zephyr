/*
 * Copyright (C) 2021 metraTec GmbH
 * Copyright (C) 2023 Linumiz 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM_XXXX_H
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM_XXXX_H

#include <zephyr/types.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIMCOM_GNSS_DATA_UTC_LEN	20
#define SIMCOM_GNSS_DATA_DATE_LEN	20

struct simcom_gnss_data {
	/**
	 * Time
	 */
	char utc[SIMCOM_GNSS_DATA_UTC_LEN];
	/**
	 * Date
	 */
	char date[SIMCOM_GNSS_DATA_DATE_LEN];
	/**
	 * Latitude in 10^-7 degree.
	 */
	float lat;
	/**
	 * Latitude direction Indicator N/S (North/South)
	*/
	char lat_dir;
	/**
	 * Longitude in 10^-7 degree.
	 */
	float lon;
	/**
	 * Longitude direction Indicator E/W (East/West)
	*/
	char lon_dir;
	/**
	 * Altitude
	 */
	float alt;
	/**
	 * Horizontal dilution of precision
	 */
	uint16_t hdop;
	/**
	 * Course over ground 
	 */
	uint16_t cog;
	/**
	 * Speed
	 */
	 float speed; 
};

/**
 * Get the modem manufacturer.
 */
const char *mdm_get_manufacturer(void);
/**
 * Get the modem model information.
 */
const char *mdm_get_model(void);
/**
 * Get the modem revision.
 */
const char *mdm_get_revision(void);
/**
 * Get the modem imei number.
 */
const char *mdm_get_imei(void);

/**
 * @brief Power on the Simcom modem.
 *
 * @return 0 on success. Otherwise -1 is returned.
 */
int mdm_simcom_power_on(void);

/**
 * @brief Power off the Simcom modem.
 *
 * @return 0 on success. Otherwise -1 is returned.
 */
int mdm_simcom_power_off(void);

/**
 * @brief Starts gnss operation.
 *
 * @return 0 on success. Otherwise <0 is returned.
 */
int mdm_simcom_start_gnss(void);

/**
 * @brief Query gnss position form the modem.
 *
 * @return 0 on success. If no fix is acquired yet -EAGAIN is returned.
 *         Otherwise <0 is returned.
 */
int mdm_simcom_query_gnss(struct simcom_gnss_data *data);

/**
 * @brief Stops gnss operation.
 *
 * @return 0 on success. Otherwise <0 is returned.
 */
int mdm_simcom_stop_gnss(void);

#endif
