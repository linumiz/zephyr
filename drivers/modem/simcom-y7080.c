/*
 * Copyright (C) 2023 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT simcom_7080

#include <stdio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_Y7080, CONFIG_MODEM_LOG_LEVEL);

#include <zephyr/drivers/modem/simcom-xxxx.h>
#include "simcom-y7080.h"

#define GNSS_SKIP_BYTES	5

int sock_create_and_connect(struct modem_context *mctx, struct modem_socket *sock,
			const struct sockaddr *addr, struct modem_cmd *cmd, size_t cmd_len)
{
	int ret;
	char cmd_buf[64] = {0};
	char ip_str[NET_IPV6_ADDR_LEN] = {0};
	uint16_t dst_port;
	struct simcom_data *mdata = (struct simcom_data *)mctx->driver_data;

	sock->id = -1;
	ret = snprintk(cmd_buf, sizeof(cmd_buf), SIMCOM_CMD_SOCKET,
					(sock->type == SOCK_STREAM) ? "STREAM" : "DGRAM",
					(sock->type == SOCK_STREAM) ? 6 : 17,
					LOCAL_LISTEN_PORT, RECEIVE_CTRL);
	if (ret < 0) {
		LOG_ERR("Failed to build connect cmd %d", ret);
		return ret;
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, cmd, cmd_len, cmd_buf,
						&mdata->unsol_resp_sem, MDM_CONNECT_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Failed to create sock %d", ret);
		return ret;
	}

	if (ret == 0) {
		sock->id = mdata->modem_sock_id;
	}

	if (sock->ip_proto == IPPROTO_UDP) {
		memcpy(&sock->dst, addr, sizeof(struct sockaddr));
		return ret;
	} else {
		ret = modem_context_sprint_ip_addr(addr, ip_str, sizeof(ip_str));
		if (ret != 0) {
			LOG_ERR("Failed to format IP!");
			return ret;
		}

		ret = modem_context_get_addr_port(addr, &dst_port);
		if (ret != 0) {
			LOG_ERR("Error getting port from IP address %d", ret);
			return ret;
		}

		memset(cmd_buf, 0, sizeof(cmd_buf));
		ret = snprintk(cmd_buf, sizeof(cmd_buf), SIMCOM_CMD_CONNECT, sock->id,
								ip_str, dst_port);
		if (ret < 0) {
			LOG_ERR("Failed to build connect cmd");
			return ret;
		}

		ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0, cmd_buf,
							&mdata->sem_response, MDM_CONNECT_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("Failed to connect %d", ret);
			return ret;
		}
	}

	return ret;
}

int send_cmd(struct modem_context *mctx, struct modem_socket *sock,
						const struct sockaddr *dest_addr,
						const char *payload, size_t len)
{
	int ret;
	char *cmd_buf = NULL;
	size_t cmd_buf_size = (len * 2) + 32;
	struct simcom_data *mdata = (struct simcom_data *)mctx->driver_data;

	cmd_buf = k_calloc(cmd_buf_size, sizeof(char));
	if (!cmd_buf) {
		return -ENOMEM;
	}

	if (sock->ip_proto == IPPROTO_UDP) {
		char ip_str[NET_IPV6_ADDR_LEN];
		uint16_t dst_port = 0;
		struct sockaddr *dst_addr = NULL;

		if (!dest_addr && sock->ip_proto == IPPROTO_UDP) {
			dst_addr = &sock->dst;
		}

		ret = modem_context_sprint_ip_addr(dst_addr, ip_str, sizeof(ip_str));
		if (ret != 0) {
			LOG_ERR("Failed to format IP!");
			goto error;
		}

		ret = modem_context_get_addr_port(dst_addr, &dst_port);
		if (ret != 0) {
			LOG_ERR("Error getting port from IP address %d", ret);
			goto error;
		}

		ret = snprintk(cmd_buf, cmd_buf_size, SIMCOM_CMD_UDP_DATA_SEND,
							sock->id, ip_str, dst_port, len);
	} else {
		ret = snprintk(cmd_buf, cmd_buf_size, SIMCOM_CMD_TCP_DATA_SEND,
								sock->id, len);
	}

	if (ret < 0) {
		LOG_ERR("Failed to build connect cmd");
		goto error;
	}

	for (int i = 0; i < len && i < MDM_MAX_DATA_LENGTH; i++) {
		int rc = snprintk(&cmd_buf[strlen(cmd_buf)], sizeof(cmd_buf), "%02x",
								((char *)payload)[i]);

		if (rc < 0) {
			goto error;
		}
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0U, cmd_buf,
							&mdata->sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Failed to send data %d ", ret);
		goto error;
	}

	mdata->current_sock_written = len;
error:
	k_free(cmd_buf);
	return ret;
}

int recv_data(struct socket_read_data *sock_data, struct modem_cmd_handler_data *data,
						char *payload, size_t payload_len,
						size_t remaining_len)
{
	int ret;

	for (int i = 0; i < payload_len; i++) {
		ret = sscanf(&payload[i * 2], "%2hhx", &sock_data->recv_buf[i]);
		if (ret < 0)
			return ret;
	}

	data->rx_buf = net_buf_skip(data->rx_buf, remaining_len);

	return ret;
}

int simcom_start_gnss(struct modem_context *mctx)
{
	int ret;
	struct simcom_data *mdata = (struct simcom_data *)mctx->driver_data;

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0U, SIMCOM_CMD_GPS_ON,
							&mdata->sem_response, K_SECONDS(2));
	if (ret < 0) {
		return -1;
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0U, SIMCOM_CMD_GPS_START,
							&mdata->sem_response, K_SECONDS(2));
	if (ret < 0) {
		return -1;
	}

	mdata->state = SIMCOM_STATE_NETWORK_AND_GNSS;

	return 0;
}

int parse_gnssinfo(char *gps_buf, struct simcom_gnss_data *gnss_data)
{
	char *saveptr;
	int ret;
	char *buf;
	int32_t number, fraction;

	for (int i = 0; i < GNSS_SKIP_BYTES; i++) {
		buf = strchr(gps_buf, ',');
		if (buf == NULL) {
			return -1;
		}
		gps_buf = buf + 1;
	}

	char *lat = gnss_get_next_param(gps_buf, ",", &saveptr);

	if (lat == NULL) {
		lat = "0";
	}

	char *lat_dir = gnss_get_next_param(NULL, ",", &saveptr);

	if (lat_dir == NULL) {
		lat_dir = "0";
	}

	char *lon = gnss_get_next_param(NULL, ",", &saveptr);

	if (lon == NULL) {
		lon = "0";
	}

	char *lon_dir = gnss_get_next_param(NULL, ",", &saveptr);

	if (lon_dir == NULL) {
		lon_dir = "0";
	}

	char *date = gnss_get_next_param(NULL, ",", &saveptr);

	if (date == NULL) {
		date = "0";
	}

	char *utc_time = gnss_get_next_param(NULL, ",", &saveptr);

	if (utc_time == NULL) {
		utc_time = "0";
	}

	char *alt = gnss_get_next_param(NULL, ",", &saveptr);

	if (alt == NULL) {
		alt = "0";
	}

	char *speed = gnss_get_next_param(NULL, ",", &saveptr);

	if (speed == NULL) {
		speed = "0";
	}

	char *course = gnss_get_next_param(NULL, ",", &saveptr);

	if (course == NULL) {
		course = "0";
	}

	gnss_skip_param(&saveptr);

	char *hdop = gnss_get_next_param(NULL, ",", &saveptr);

	if (hdop == NULL) {
		hdop = "0";
	}

	gnss_data->lat = atof(lat);
	gnss_data->lat_dir = lat_dir[0];
	gnss_data->lon = atof(lon);
	gnss_data->lon_dir = lon_dir[0];
	if (alt) {
		gnss_data->alt =  atof(alt);
	} else {
		gnss_data->alt = 0;
	}

	strcpy(gnss_data->date, date);

	strcpy(gnss_data->utc, utc_time);

	if (speed) {
		gnss_data->speed = atof(speed);
	}

	gnss_data->hdop = atoi(hdop);

	if (course) {
		gnss_data->cog = atoi(course);
	}

	return 0;
}
