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
#define SIMCOM_SMS_MAX_LEN 160

struct simcom_gnss_data {
	/**
	 * Whether gnss is powered or not.
	 */
	bool run_status;
	/**
	 * Whether fix is acquired or not.
	 */
	bool fix_status;
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
 * Possible sms states in memory.
 */
enum simcom_sms_stat {
	SIMCOM_SMS_STAT_REC_UNREAD = 0,
	SIMCOM_SMS_STAT_REC_READ,
	SIMCOM_SMS_STAT_STO_UNSENT,
	SIMCOM_SMS_STAT_STO_SENT,
	SIMCOM_SMS_STAT_ALL,
};

/**
 * Possible ftp return codes.
 */
enum simcom_ftp_rc {
	/* Operation finished correctly. */
	SIMCOM_FTP_RC_OK = 0,
	/* Session finished. */
	SIMCOM_FTP_RC_FINISHED,
	/* An error occurred. */
	SIMCOM_FTP_RC_ERROR,
};

/**
 * Buffer structure for sms.
 */
struct simcom_sms {
	/* First octet of the sms. */
	uint8_t first_octet;
	/* Message protocol identifier. */
	uint8_t tp_pid;
	/* Status of the sms in memory. */
	enum simcom_sms_stat stat;
	/* Index of the sms in memory. */
	uint16_t index;
	/* Time the sms was received. */
	struct {
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t timezone;
	} time;
	/* Buffered sms. */
	char data[SIMCOM_SMS_MAX_LEN + 1];
	/* Length of the sms in buffer. */
	uint8_t data_len;
};

/**
 * Buffer structure for sms reads.
 */
struct simcom_sms_buffer {
	/* sms structures to read to. */
	struct simcom_sms *sms;
	/* Number of sms structures. */
	uint8_t nsms;
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
 * @brief Starts the modem in network operation mode.
 *
 * @return 0 on success. Otherwise <0 is returned.
 */
int mdm_simcom_start_network(void);

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

/**
 * Read sms from sim module.
 *
 * @param buffer Buffer structure for sms.
 * @return Number of sms read on success. Otherwise -1 is returned.
 *
 * @note The buffer structure needs to be initialized to
 * the size of the sms buffer. When this function finishes
 * successful, nsms will be set to the number of sms read.
 * If the whole structure is filled a subsequent read may
 * be needed.
 */
int mdm_simcom_read_sms(struct simcom_sms_buffer *buffer);

/**
 * Delete a sms at a given index.
 *
 * @param index The index of the sms in memory.
 * @return 0 on success. Otherwise -1 is returned.
 */
int mdm_simcom_delete_sms(uint16_t index);

/**
 * Start a ftp get session.
 *
 * @param server The ftp servers address.
 * @param user User name for the ftp server.
 * @param passwd Password for the ftp user.
 * @param file File to be downloaded.
 * @param path Path to the file on the server.
 * @return 0 if the session was started. Otherwise -1 is returned.
 */
int mdm_simcom_ftp_get_start(const char *server, const char *user, const char *passwd,
						const char *file, const char *path);

/**
 * Read data from a ftp get session.
 *
 * @param dst The destination buffer.
 * @param size Initialize to the size of dst. Gets set to the number
 *             of bytes actually read.
 * @return According simcom_ftp_rc.
 */
int mdm_simcom_ftp_get_read(char *dst, size_t *size);
#endif
