/*
 * Copyright (C) 2023 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT simcom

#include "simcom-xxxx.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_Y7080, CONFIG_MODEM_LOG_LEVEL);

#include "simcom-y7080.h"

#define GNSS_SKIP_BYTES 	5

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

	ret = snprintk(buf, sizeof(buf), "AT+NSOCR= \"%s\", %d, %d, %d, %d, \"%s\"", protocol_type, protocol, 0, sock->sock_fd,
			receive_ctrl, af_type);
	if (ret < 0) {
		LOG_ERR("Failed to build connect command. ID: %d, FD: %d", sock->id, sock->sock_fd);
		return -ENOMEM;
	}

	return 0;
}

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
	char *buf;
	int32_t number, fraction;

	for(int i = 0; i < GNSS_SKIP_BYTES; i++){
		buf = strchr(gps_buf, ',');
		if(buf == NULL){
			return -1;
		}
		gps_buf = buf + 1;
	}

	char *lat = gnss_get_next_param(gps_buf, ",", &saveptr);
	if (lat == NULL) {
		lat = "0";
	}

	char *lat_dir = gnss_get_next_param(NULL, ",", &saveptr);
	if(lat_dir == NULL) {
		lat_dir = "0";
	}

	char *lon = gnss_get_next_param(NULL, ",", &saveptr);
	if (lon == NULL) {
		lon = "0";
	}

	char *lon_dir = gnss_get_next_param(NULL, ",", &saveptr);
	if(lon_dir == NULL) {
		lon_dir = "0";
	}

	char *date = gnss_get_next_param(NULL, ",", &saveptr);
	if(date == NULL){
		date = "0";
	}

	char *utc_time = gnss_get_next_param(NULL, ",", &saveptr);
	if(utc_time == NULL){
		utc_time = "0";
	}

	char *alt = gnss_get_next_param(NULL, ",", &saveptr);
	if(alt == NULL){
		alt = "0";
	}

	char *speed = gnss_get_next_param(NULL, ",", &saveptr);
	if(speed == NULL){
		speed = "0";
	}

	char *course = gnss_get_next_param(NULL, ",", &saveptr);
	if(course == NULL){
		course = "0";
	}

	gnss_skip_param(&saveptr);

	char *hdop = gnss_get_next_param(NULL, ",", &saveptr);
	if (hdop == NULL) {
		hdop = "0";
	}

	gnss_data->lat = get_float_from_str(lat);
	gnss_data->lat_dir = lat_dir[0];
	gnss_data->lon = get_float_from_str(lon) ;
	gnss_data->lon_dir = lon_dir[0];
	if (alt) {
		gnss_data->alt =  get_float_from_str(alt);;
	} else {
		gnss_data->alt = 0;
	}

	strcpy(gnss_data->date, date);

	strcpy(gnss_data->utc, utc_time);

	if(speed)
		gnss_data->speed = get_float_from_str(speed);

	gnss_data->hdop = get_int_from_str(hdop);

	if(course)
		gnss_data->cog = get_int_from_str(course);

	return 0;
}

