/*
 * copyright (c) 2023 Linumiz
 *
 * spdx-license-identifier: apache-2.0
 */

#ifndef SIMCOM_Y7080_H
#define SIMCOM_Y7080_H

#include "simcom-xxxx.h"
#include <zephyr/drivers/modem/simcom-xxxx.h>

#define MDM_MAX_DATA_SEND_LENGTH	1400
#define MDM_MAX_DATA_RECV_LENGTH	512

/* SETUP CMDS */
#define SIMCOM_SET_BANDS	SETUP_CMD_NOHANDLE("AT+NBAND=" MDM_LTE_BANDS)

/* SIM CMDS */
#define SIMCOM_CCID_CMD		SETUP_CMD("AT+NCCID", "+NCCID:", on_cmd_ccid, 1U, "")
#define SIMCOM_CIMI_CMD		SETUP_CMD("AT+CIMI", "+CIMI:", on_cmd_cimi, 1U, "")
#define SIMCOM_CPIN_CMD		SETUP_CMD("AT+CPIN?", "+CPIN:", on_cmd_cpin, 1U, "")

#define SIMCOM_CSQ_CMD		MODEM_CMD("+CSQ:", on_cmd_csq, 2U, ",")

/* GNSS CMDS */
#define SIMCOM_CMD_GPS_ON		"AT+CGNSSPWR=1"
#if defined CONFIG_SIMCOM_GNSS_WARM_START
#define SIMCOM_CMD_GPS_START		"AT+CGPSWARM"
#elif CONFIG_SIMCOM_GNSS_HOT_START
#define SIMCOM_CMD_GPS_START		"AT+CGPSHOT"
#else
#define SIMCOM_CMD_GPS_START		"AT+CGPSCOLD"
#endif

#define SIMCOM_CMD_GPS_STOP		"AT+CGNSSPWR=0"
#define SIMCOM_CMD_GPS_INFO		"AT+CGNSSINFO"
#define SIMCOM_CMD_MDM_GPS_INFO		MODEM_CMD("+CGNSSINFO:", on_cmd_gnssinfo, 0U, NULL)

/* DNS CMDS */
#define SIMCOM_CMD_DNS_MDM_CMD		MODEM_CMD("+XDNS:", on_cmd_dns_resolve, 1U, "")
#define SIMCOM_CMD_DNS_RESOLVE		"AT+XDNS=%s"

/* Socket CMDS */
/* Offload connect CMDs */
#define SIMCOM_CMD_SOCKET		"AT+NSOCR=%s,%d,%d,%d"
#define SIMCOM_CMD_CONNECT_MDM_CMD	MODEM_CMD("+NSOCR:", on_cmd_offload_connect, 1U, "")
#define SIMCOM_CMD_CONNECT		"AT+NSOCO=%d,%s,%d"
#define CONNECT_ARGS			0
#define SIMCOM_CMD_SOCKET_STAT_MDM_CMD	MODEM_CMD("+NSOCLI:", on_urc_socket_status, 1U, "")
#define SIMCOM_CMD_SOCKET_CLOSE		"AT+NSOCL=%d"

/* Offload send and recv CMDs */
#define LOCAL_LISTEN_PORT		0
#define RECEIVE_CTRL			1
#define SIMCOM_CMD_TCP_DATA_SEND	"AT+NSOSD=%d,%d,"
#define SIMCOM_CMD_UDP_DATA_SEND	"AT+NSOST=%d,%s,%d,%d,"
#define SIMCOM_CMD_DATA_RECV		"AT+NSORF=%d,%d"
#define SIMCOM_CMD_MDM_RECV_CMD		MODEM_CMD("+NSORF:", on_cmd_recvfrom, 6U, ",")
#define SIMCOM_CMD_PDP_ACTIVATE		"AT+CGACT=0,1"
#define SIMCOM_CMD_DATA_REPORTING	"AT+NSONMI=1"

/* Unsoliciated CMDs */
#define SIMCOM_CMD_REPORT_DOWN_DATA_MDM_CMD	MODEM_CMD("+NSONMI:", on_urc_dataready, 2U, ",")
#define DATA_LEN_ARGS				3
#define DATA_ARGS				4
#define SOCK_ID_ARGS				0
#define SIMCOM_CMD_PDP_ACTIVE_CMD		MODEM_CMD("+CGEV:", on_pdp_active, 3U, " ")
#define PDP_RESP_STR				strcmp(argv[2], "ACT")

#endif
