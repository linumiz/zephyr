/*
 * Copyright (C) 2021 metraTec GmbH
 * copyright (c) 2023 Linumiz
 *
 * spdx-license-identifier: apache-2.0
 */

#ifndef SIMCOM_SIM7080_H
#define SIMCOM_SIM7080_H

#include "simcom-xxxx.h"
#include <zephyr/drivers/modem/simcom-xxxx.h>

#define SIMCOM_CMD_DEFINE(rsp, on_rsp_callback, args_no, delimiter) MODEM_CMD(rsp, on_rsp_callback, args_no, delimiter)

/* GNSS CMDS */
#define SIMCOM_CMD_GPS_ON			"AT+CGNSPWR=1"
#define SIMCOM_CMD_GPS_START			"AT+CGPSCOLD"

#define SIMCOM_CMD_GPS_STOP			"AT+CGNSPWR=0"
#define SIMCOM_CMD_GPS_INFO			"AT+CGNSINF"
#define SIMCOM_CMD_MDM_GPS_INFO 		SIMCOM_CMD_DEFINE("+CGNSINF: ", on_cmd_gnssinfo, 0U, NULL)

/* DNS CMDS */
#define SIMCOM_CMD_DNS_MDM_CMD			SIMCOM_CMD_DEFINE("+CDNSGIP: ", on_cmd_dns_resolve, 2U, ",")
#define SIMCOM_CMD_DNS_RESOLVE			"AT+CDNSGIP=%s, 10, 20000"

/* Socket CMDS */
/* Offload connect CMDs */
#define SIMCOM_CMD_CONNECT			"AT+CAOPEN=0,%d,\"%s\",\"%s\",%d"
#define SIMCOM_CMD_CONNECT_MDM_CMD		SIMCOM_CMD_DEFINE("+CAOPEN: ", on_cmd_offload_connect, 2U, "")
#define CONNECT_ARGS				1

/* Offload send and recv CMDs */
#define SIMCOM_CMD_SOCKET_CLOSE 		"AT+NSOCL=%d"
#define SIMCOM_CMD_DATA_SEND 			"AT+CASEND=%d,%ld"
#define SIMCOM_CMD_DATA_RECV			"AT+CARECV=%d,%zd"
#define SIMCOM_CMD_MDM_RECV_CMD 		SIMCOM_CMD_DEFINE("+CARECV: ", on_cmd_recvfrom, 1U, "")
#define SIMCOM_CMD_PDP_ACTIVATE 		"AT+CNACT=0,1"

/* Unsoliciated CMDs */
#define SIMCOM_CMD_SOCKET_STAT_MDM_CMD		SIMCOM_CMD_DEFINE("+CASTATE: ", on_cmd_socket_status, 2U, ",")
#define SIMCOM_CMD_REPORT_DOWN_DATA_MDM_CMD	SIMCOM_CMD_DEFINE("+CADATAIND: ", on_urc_dataready, 1U, "")
#define DATA_LEN_ARGS				0
#define SIMCOM_CMD_PDP_ACTIVE_CMD		SIMCOM_CMD_DEFINE("+APP PDP: ", on_pdp_active, 2U, ",")
#define PDP_RESP_STR				strcmp(argv[1], "ACTIVE")

int parse_gnssinfo(char *buf, struct simcom_gnss_data *gnss_data);

#endif
