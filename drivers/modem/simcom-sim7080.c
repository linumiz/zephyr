/*
 * Copyright (C) 2021 metraTec GmbH
 * Copyright (C) 2023 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_sim7080, CONFIG_MODEM_LOG_LEVEL);

#include "simcom-sim7080.h"


int simcom_start_gnss(struct modem_context *mctx)
{
	int ret;
	struct simcom_data *mdata = (struct simcom_data *)mctx->driver_data;

	mdata->state = SIMCOM_STATE_INIT;
	k_work_cancel_delayable(&mdata->rssi_query_work);

	ret = modem_autobaud();
	if (ret < 0) {
		LOG_ERR("Failed to start modem!!");
		return -1;
	}

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

	mdata->state = SIMCOM_STATE_GNSS;

	return 0;
}

/**
 * Parses cgnsinf response into the gnss_data structure.
 *
 * @param gps_buf Null terminated buffer containing the response.
 * @return 0 on successful parse. Otherwise <0 is returned.
 */
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
		utc = "0";
	}

	char *lat = gnss_get_next_param(NULL, ",", &saveptr);

	if (lat == NULL) {
		lat = "0";
	}

	char *lon = gnss_get_next_param(NULL, ",", &saveptr);

	if (lon == NULL) {
		lon = "0";
	}

	char *alt = gnss_get_next_param(NULL, ",", &saveptr);
	char *speed = gnss_get_next_param(NULL, ",", &saveptr);
	char *course = gnss_get_next_param(NULL, ",", &saveptr);

	/* discard fix mode and reserved*/
	gnss_skip_param(&saveptr);
	gnss_skip_param(&saveptr);

	char *hdop = gnss_get_next_param(NULL, ",", &saveptr);

	if (hdop == NULL) {
		hdop = "0";
	}

	gnss_data->run_status = 1;
	gnss_dat->fix_status = 1;
	gnss_data->lat = atof(lat);
	gnss_data->lon = atof(lon);
	if (alt) {
		gnss_data->alt =  atof(alt);
	} else {
		gnss_data->alt = 0;
	}

	strcpy(gnss_data->utc, utc);
	if (speed) {
		gnss_data->speed = atof(speed);
	}

	gnss_data->hdop = atoi(hdop);

	if (course) {
		gnss_data->cog = atoi(course);
	}

	return 0;
error:
	memset(&gnss_data, 0, sizeof(gnss_data));
	return -1;
}

int sock_create_and_connect(struct modem_context *mctx, struct modem_socket *sock,
					const struct sockaddr *addr,
					struct modem_cmd *cmd, size_t cmd_len)
{
	int ret;
	char cmd_buf[sizeof("AT+CAOPEN: #,#,#####,#xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx#,####")];
	char ip_str[NET_IPV6_ADDR_LEN];
	uint16_t dst_port;
	struct simcom_data *mdata = (struct simcom_data *)mctx->driver_data;

	ret = modem_context_sprint_ip_addr(addr, ip_str, sizeof(ip_str));
	if (ret != 0) {
		LOG_ERR("Failed to format IP!");
		goto exit;
	}

	ret = modem_context_get_addr_port(addr, &dst_port);
	if (ret != 0) {
		LOG_ERR("Error getting port from IP address %d", ret);
		goto exit;
	}

	ret = snprintk(cmd_buf, sizeof(cmd_buf), SIMCOM_CMD_CONNECT, sock->id,
						(sock->type == SOCK_STREAM) ? "TCP" : "UDP",
						ip_str, dst_port);
	if (ret < 0) {
		LOG_ERR("Failed to build connect cmd");
		errno = ENOMEM;
		goto exit;
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, cmd, cmd_len, cmd_buf,
					&mdata->unsol_resp_sem, MDM_CONNECT_TIMEOUT);

	if (ret < 0) {
		goto exit;
	}

	return ret;

exit:
	sock->id = -1;
	return ret;
}

int send_cmd(struct modem_context *mctx, struct modem_socket *sock,
					const struct sockaddr *dest_addr,
					const char *payload, size_t len)
{
	int ret;
	char send_buf[sizeof("AT+CASEND=#,####")] = { 0 };
	char ctrlz = 0x1A;
	struct simcom_data *mdata = (struct simcom_data *)mctx->driver_data;

	ret = snprintk(send_buf, sizeof(send_buf), "AT+CASEND=%d,%ld", sock->id, (long)len);
	if (ret < 0) {
		return ret;
	}

	/* Make sure only one send can be done at a time. */
	k_sem_take(&mdata->cmd_handler_data.sem_tx_lock, K_FOREVER);
	k_sem_reset(&mdata->sem_tx_ready);

	/* Send CASEND */
	mdata->current_sock_written = len;
	ret = modem_cmd_send_nolock(&mctx->iface, &mctx->cmd_handler, NULL, 0U, send_buf, NULL,
				    K_NO_WAIT);
	if (ret < 0) {
		goto exit;
	}

	/* Wait for '> ' */
	ret = k_sem_take(&mdata->sem_tx_ready, K_SECONDS(2));
	if (ret < 0) {
		goto exit;
	}

	/* Send data */
	mctx->iface.write(&mctx->iface, payload, len);
	mctx->iface.write(&mctx->iface, &ctrlz, 1);

	/* Wait for the OK */
	k_sem_reset(&mdata->sem_response);
	ret = k_sem_take(&mdata->sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Timeout waiting for OK");
	}

exit:
	k_sem_give(&mdata->cmd_handler_data.sem_tx_lock);
	/* Data was successfully sent */

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return ret;
}

int recv_data(struct socket_read_data *sock_data, struct modem_cmd_handler_data *data,
					char *payload, size_t payload_len, size_t remaining_len)
{
	int ret;

	ret = net_buf_linearize(sock_data->recv_buf, sock_data->recv_buf_len, data->rx_buf, 0,
				(uint16_t)payload_len);

	data->rx_buf = net_buf_skip(data->rx_buf, ret);

	return ret;
}

int simcom_ftp_get_read(struct modem_context *mctx, struct modem_cmd *cmds,
					size_t cmd_size, char *dst, size_t *size)
{
	int ret;
	char buffer[sizeof("AT+FTPGET=#,######")];
	struct simcom_data *mdata = (struct simcom_data *)mctx->driver_data;

	/* Some error occurred. */
	if (mdata->ftp.state == SIMCOM_FTP_CONNECTION_STATE_ERROR ||
	    mdata->ftp.state == SIMCOM_FTP_CONNECTION_STATE_INITIAL) {
		return SIMCOM_FTP_RC_ERROR;
	}

	/* Setup buffer. */
	mdata->ftp.read_buffer = dst;
	mdata->ftp.nread = *size;

	/* Read ftp data. */
	ret = snprintk(buffer, sizeof(buffer), "AT+FTPGET=2,%zu", *size);
	if (ret < 0) {
		*size = 0;
		return SIMCOM_FTP_RC_ERROR;
	}

	/* Wait for data from the server. */
	k_sem_take(&mdata->sem_ftp, K_MSEC(200));

	if (mdata->ftp.state == SIMCOM_FTP_CONNECTION_STATE_FINISHED) {
		*size = 0;
		return SIMCOM_FTP_RC_FINISHED;
	} else if (mdata->ftp.state == SIMCOM_FTP_CONNECTION_STATE_ERROR) {
		*size = 0;
		return SIMCOM_FTP_RC_ERROR;
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, cmds, cmd_size, buffer,
			     &mdata->sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		*size = 0;
		return SIMCOM_FTP_RC_ERROR;
	}

	/* Set read size. */
	*size = mdata->ftp.nread;

	return SIMCOM_FTP_RC_OK;
}

int simcom_ftp_get_start(struct modem_context *mctx, const char *server,
					const char *user, const char *passwd,
					const char *file, const char *path)
{
	int ret;
	char buffer[256];
	struct simcom_data *mdata = (struct simcom_data *)mctx->driver_data;

	/* Start network. */
	ret = mdm_simcom_start_network();
	if (ret < 0) {
		LOG_ERR("Failed to start network for FTP!");
		return -1;
	}

	/* Set connection id for ftp. */
	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0U, "AT+FTPCID=0",
			     &mdata->sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("Failed to set FTP Cid!");
		return -1;
	}

	/* Set ftp server. */
	ret = snprintk(buffer, sizeof(buffer), "AT+FTPSERV=\"%s\"", server);
	if (ret < 0) {
		LOG_WRN("Failed to build command!");
		return -1;
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0U, buffer,
						&mdata->sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("Failed to set FTP Cid!");
		return -1;
	}

	/* Set ftp user. */
	ret = snprintk(buffer, sizeof(buffer), "AT+FTPUN=\"%s\"", user);
	if (ret < 0) {
		LOG_WRN("Failed to build command!");
		return -1;
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0U, buffer,
						&mdata->sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("Failed to set ftp user!");
		return -1;
	}

	/* Set ftp password. */
	ret = snprintk(buffer, sizeof(buffer), "AT+FTPPW=\"%s\"", passwd);
	if (ret < 0) {
		LOG_WRN("Failed to build command!");
		return -1;
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0U, buffer,
						&mdata->sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("Failed to set ftp password!");
		return -1;
	}

	/* Set ftp filename. */
	ret = snprintk(buffer, sizeof(buffer), "AT+FTPGETNAME=\"%s\"", file);
	if (ret < 0) {
		LOG_WRN("Failed to build command!");
		return -1;
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0U, buffer,
						&mdata->sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("Failed to set ftp filename!");
		return -1;
	}

	/* Set ftp filename. */
	ret = snprintk(buffer, sizeof(buffer), "AT+FTPGETNAME=\"%s\"", file);
	if (ret < 0) {
		LOG_WRN("Failed to build command!");
		return -1;
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0U, buffer,
						&mdata->sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("Failed to set ftp filename!");
		return -1;
	}

	/* Set ftp path. */
	ret = snprintk(buffer, sizeof(buffer), "AT+FTPGETPATH=\"%s\"", path);
	if (ret < 0) {
		LOG_WRN("Failed to build command!");
		return -1;
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0U, buffer,
						&mdata->sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("Failed to set ftp path!");
		return -1;
	}

	/* Initialize ftp variables. */
	mdata->ftp.read_buffer = NULL;
	mdata->ftp.nread = 0;
	mdata->ftp.state = SIMCOM_FTP_CONNECTION_STATE_INITIAL;

	/* Start the ftp session. */
	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0U, "AT+FTPGET=1",
						&mdata->sem_ftp, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("Failed to start session!");
		return -1;
	}

	if (mdata->ftp.state != SIMCOM_FTP_CONNECTION_STATE_CONNECTED) {
		LOG_WRN("Session state is not connected!");
		return -1;
	}

	return 0;
}

int simcom_read_sms(struct modem_context *mctx, struct modem_cmd *cmds,
			size_t cmd_size, struct simcom_sms_buffer *buffer)
{
	int ret;
	struct simcom_data *mdata = (struct simcom_data *)mctx->driver_data;

	mdata->sms_buffer = buffer;
	mdata->sms_buffer_pos = 0;

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, cmds, cmd_size, "AT+CMGL=4",
						&mdata->sem_response, K_SECONDS(20));
	if (ret < 0) {
		return -1;
	}

	return mdata->sms_buffer_pos;
}

int simcom_delete_sms(struct modem_context *mctx, uint16_t index)
{
	int ret;
	char buf[sizeof("AT+CMGD=#####")] = { 0 };
	struct simcom_data *mdata = (struct simcom_data *)mctx->driver_data;

	ret = snprintk(buf, sizeof(buf), "AT+CMGD=%u", index);
	if (ret < 0) {
		return -1;
	}

	ret = modem_cmd_send(&mctx->iface, &mctx->cmd_handler, NULL, 0, buf,
						&mdata->sem_response, K_SECONDS(5));
	if (ret < 0) {
		return -1;
	}

	return 0;
}
