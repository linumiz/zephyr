/*
 * Copyright (C) 2023 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM_Y7080_H
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM_Y7080_H

#include <zephyr/types.h>

#include <stdint.h>

#define Y7080_GNSS_DATA_UTC_LEN 	sizeof("hhmmss.s")
#define Y7080_GNSS_DATA_DATE_LEN	sizeof("ddmmyy")

struct y7080_gnss_data {
	/**
	 * Date in format of ddmmyy.
	 */
	char date[Y7080_GNSS_DATA_DATE_LEN];
	/**
	 * UTC time in format hhmmss.s.
	 */
	char utc[Y7080_GNSS_DATA_UTC_LEN];
	/**
	 * Latitude.
	 */
	float lat;
	/**
	 * Latitude Indicator. N for North, S for South.
	 */
	char lat_dir;
	/**
	 * Longitude.
	 */
	float lon;
	/**
	 * Longitudee Indicator. E for East, W for West.
	 */
	char lon_dir;
	/**
	 * Altitude in m.
	 */
	float alt;
	/**
	 * Horizontal dilution of precision.
	 */
	uint16_t hdop;
	/**
	 * Course over ground in degrees.
	 */
	uint16_t cog;
	/**
	 * Speed in knots.
	 */
	uint16_t knots;
};

/**
 * @brief starts gnss operation.
 *
 * @return 0 on success. otherwise <0 is returned.
 */
int mdm_y7080_start_gnss(void);

/**
 * @brief stops gnss operation.
 *
 * return 0 on success. Otherwise err code is returned.
 */
int mdm_y7080_stop_gnss(void);

/**
 * Get the y7080 manufacturer.
 */
const char *mdm_y7080_get_manufacturer(void);

/**
 * Get the y7080 model information.
 */
const char *mdm_y7080_get_model(void);

/**
 * Get the y7080 revision.
 */
const char *mdm_y7080_get_revision(void);

/**
 * Get the y7080 imei number.
 */
const char *mdm_y7080_get_imei(void);

#endif
