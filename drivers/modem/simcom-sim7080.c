/*
 * Copyright (C) 2021 metraTec GmbH
 * Copyright (C) 2023 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT simcom

#include "simcom-xxxx.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_sim7080, CONFIG_MODEM_LOG_LEVEL);

#include "simcom-sim70801.h"

#if 0
int offload_connect_cmd(struct modem_socket *sock, const struct sockaddr *addr, char *buf)
{
	char *protocol_type;
	uint8_t protocol;
	uint8_t receive_ctrl;
	char af_type[] = { 0 };
	int ret;

	/* get the destination port */
	if (addr->sa_family == AF_INET6) {
		strcpy(af_type, "AF_INET6");
	} else if (addr->sa_family == AF_INET) {
		strcpy(af_type, "AF_INET");
	}

	/* Get protocol */
	protocol_type = (sock->type == SOCK_STREAM) ? "STREAM" : "DGRAM";
	protocol = (sock->type == SOCK_STREAM) ? 6 : 17;
	receive_ctrl = 1; /* Receive control */

	ret = snprintk(buf, sizeof(buf), "AT+NSOCR= %s, %d, %d, %d, %d, %s", protocol_type, protocol, 0, sock->sock_fd,
			receive_ctrl, af_type);
	if (ret < 0) {
		LOG_ERR("Failed to build connect command. ID: %d, FD: %d", sock->id, sock->sock_fd);
		return -ENOMEM;
	}

	return 0;
}
#endif

static float get_float_from_str(char *buf)
{
	return atof(buf);
}

static int get_int_from_str(char *buf)
{
	return atoi(buf);
}

int parse_gnssinfo(char *gps_buf, struct simcom_gnss_data *gnss_data)
{
	char *saveptr;
	int ret;
	int32_t number, fraction;

	char *run_status = gnss_get_next_param(gps_buf, ",", &saveptr);
	if (run_status == NULL) {
		goto error;
	} else if (*run_status != '1') {
		goto error;
	}

	char *fix_status = gnss_get_next_param(NULL, ",", &saveptr);
	if (fix_status == NULL) {
		goto error;
	} else if (*fix_status != '1') {
		goto error;
	}

	char *utc = gnss_get_next_param(NULL, ",", &saveptr);
	if (utc == NULL) {
		goto error;
	}

	char *lat = gnss_get_next_param(NULL, ",", &saveptr);
	if (lat == NULL) {
		goto error;
	}

	char *lon = gnss_get_next_param(NULL, ",", &saveptr);
	if (lon == NULL) {
		goto error;
	}

	char *alt = gnss_get_next_param(NULL, ",", &saveptr);
	char *speed = gnss_get_next_param(NULL, ",", &saveptr);
	char *course = gnss_get_next_param(NULL, ",", &saveptr);

	/* discard fix mode and reserved*/
	gnss_skip_param(&saveptr);
	gnss_skip_param(&saveptr);

	char *hdop = gnss_get_next_param(NULL, ",", &saveptr);
	if (hdop == NULL) {
		goto error;
	}

	gnss_data->lat = get_float_from_str(lat);
        gnss_data->lon = get_float_from_str(lon);
        if (alt) {
                gnss_data->alt =  get_float_from_str(alt);
        } else {
                gnss_data->alt = 0;
        }

	strcpy(gnss_data->utc, utc);
	if(speed)
                gnss_data->speed = get_float_from_str(speed);

	gnss_data->hdop = get_int_from_str(hdop);

	if(course)
                gnss_data->cog = get_int_from_str(course);

	return 0;
error:
	memset(&gnss_data, 0, sizeof(gnss_data));
	return -1;
}
