/*
 * copyright (c) 2023 Linumiz
 *
 * spdx-license-identifier: apache-2.0
 */

#ifndef SIMCOM_Y7080_H
#define SIMCOM_Y7080_H

#include "simcom-xxxx.h"
#include <zephyr/drivers/modem/simcom-xxxx.h>

/* SETUP CMDS */
#define SIMCOM_SET_BANDS 			SIMCOM_SETUP_CMD_NO_HANDLE("AT+NBAND=MDM_LTE_BANDS")

/* GNSS CMDS */
#define SIMCOM_CMD_GPS_ON			"AT+CGNSSPWR=1"
#if defined CONFIG_SIMCOM_GNSS_WARM_START
#define SIMCOM_CMD_GPS_START			"AT+CGPSWARM"
#elif CONFIG_SIMCOM_GNSS_HOT_START
#define SIMCOM_CMD_GPS_START			"AT+CGPSHOT"
#else
#define SIMCOM_CMD_GPS_START			"AT+CGPSCOLD"
#endif

#define SIMCOM_CMD_GPS_STOP			"AT+CGNSSPWR=0"
#define SIMCOM_CMD_GPS_INFO			"AT+CGNSSINFO"
#define SIMCOM_CMD_MDM_GPS_INFO 		SIMCOM_CMD_DEFINE("+CGNSSINFO:", on_cmd_gnssinfo, 0U, NULL)

/* DNS CMDS */
#define SIMCOM_CMD_DNS_MDM_CMD			SIMCOM_CMD_DEFINE("+xdns:", on_cmd_dns_resolve, 1U, "")
#define SIMCOM_CMD_DNS_RESOLVE			"AT+XDNS=%s"

/* Socket CMDS */
/* Offload connect CMDs */
#define SIMCOM_CMD_CONNECT			"AT+NSOCO=%d, \"%s\", %d"
#define SIMCOM_CMD_CONNECT_MDM_CMD		SIMCOM_CMD_DEFINE("+NSOCR:", on_cmd_offload_connect, 1U, "")
#define CONNECT_ARGS				0
#define SIMCOM_CMD_SOCKET_STAT_MDM_CMD		SIMCOM_CMD_DEFINE("+NSOCLI:", on_cmd_socket_status, 1U, "")
#define SIMCOM_CMD_SOCKET_CLOSE 		"AT+NSOCL=%d"

/* Offload send and recv CMDs */
#define SIMCOM_CMD_DATA_SEND			"AT+NSOSD=%d, %ld"
#define SIMCOM_CMD_DATA_RECV			"AT+NSORF=%d, %zd"
#define SIMCOM_CMD_MDM_RECV_CMD 		SIMCOM_CMD_DEFINE("+NSORF:", on_cmd_recvfrom, 6U, ",")
#define SIMCOM_CMD_PDP_ACTIVATE 		"AT+CGACT=0, 1"
#define SIMCOM_CMD_DATA_REPORTING		"AT+NSONMI=1"

/* Unsoliciated CMDs */
#define SIMCOM_CMD_REPORT_DOWN_DATA_MDM_CMD	SIMCOM_CMD_DEFINE("+NSONMI:", on_urc_dataready, 2U, ",")
#define DATA_LEN_ARGS				3
#define SIMCOM_CMD_PDP_ACTIVE_CMD		SIMCOM_CMD_DEFINE("+CGEV:", on_pdp_active, 3U, " ")
#define PDP_RESP_STR				strcmp(argv[2], "ACT")

int offload_connect_cmd(struct modem_socket *sock, const struct sockaddr *addr, char *buf);
int parse_gnssinfo(char *buf, struct simcom_gnss_data *gnss_data);

#endif
