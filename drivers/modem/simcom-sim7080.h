/*
 * Copyright (C) 2021 metraTec GmbH
 * copyright (c) 2023 Linumiz
 *
 * spdx-license-identifier: apache-2.0
 */

#ifndef SIMCOM_SIM7080_H
#define SIMCOM_SIM7080_H

#include <zephyr/drivers/modem/simcom-xxxx.h>
#include "simcom-xxxx.h"

#define MDM_MAX_DATA_SEND_LENGTH	1024
#define MDM_MAX_DATA_RECV_LENGTH	MDM_MAX_DATA_SEND_LENGTH

/* SETUP CMDS */
/* LTE-M CMDS */
#if defined CONFIG_MODEM_SIMCOM_RAT_M1

#define SIMCOM_MODE_SELECTION			SETUP_CMD_NOHANDLE("AT+CNMP=38")
#define SIMCOM_MODE_SELECTION_RAT		SETUP_CMD_NOHANDLE("AT+CMNB=1")
#define SIMCOM_SET_BANDS	SIMCOM_MODE_SELECTION, SIMCOM_MODE_SELECTION_RAT, \
		SETUP_CMD_NOHANDLE("AT+CBANDCFG=\"CAT-M\"," MDM_LTE_BANDS)

#endif

/* NB_IoT CMDS */
#if defined CONFIG_MODEM_SIMCOM_RAT_NB1
#define SIMCOM_MODE_SELECTION			SETUP_CMD_NOHANDLE("AT+CNMP=38")
#define SIMCOM_MODE_SELECTION_RAT		SETUP_CMD_NOHANDLE("AT+CMNB=2")
#define SIMCOM_SET_BANDS	SIMCOM_MODE_SELECTION, SIMCOM_MODE_SELECTION_RAT, \
		SETUP_CMD_NOHANDLE("AT+CBANDCFG=\"NB-IOT\"," MDM_LTE_BANDS)


#endif

/* GSM CMDS */
#if defined CONFIG_MODEM_SIMCOM_RAT_GSM
#define SIMCOM_MODE_SELECTION_GSM		SETUP_CMD_NOHANDLE("AT+CNMP=13")
#endif

/* SIM CMDS */
#define SIMCOM_CCID_CMD			SETUP_CMD("AT+CCID", "", on_cmd_ccid, 0U, "")
#define SIMCOM_CIMI_CMD			SETUP_CMD("AT+CIMI?", "", on_cmd_cimi, 0U, "")

#define SIMCOM_CPIN_CMD			SETUP_CMD("AT+CPIN?", "+CPIN: ", on_cmd_cpin, 1U, "")
#define SIMCOM_CSQ_CMD			MODEM_CMD("+CSQ: ", on_cmd_csq, 2U, ",")


/* GNSS CMDS */
#define SIMCOM_CMD_GPS_ON			"AT+CGNSPWR=1"
#define SIMCOM_CMD_GPS_START			"AT+CGNSCOLD"
#define SIMCOM_CMD_GPS_STOP			"AT+CGNSPWR=0"
#define SIMCOM_CMD_GPS_INFO			"AT+CGNSINF"
#define SIMCOM_CMD_MDM_GPS_INFO		\
		MODEM_CMD("+CGNSINF: ", on_cmd_gnssinfo, 0U, NULL)

/* DNS CMDS */
#define SIMCOM_CMD_DNS_MDM_CMD		\
		MODEM_CMD("+CDNSGIP: ", on_cmd_dns_resolve, 2U, ",")

#define SIMCOM_CMD_DNS_RESOLVE			"AT+CDNSGIP=%s, 10, 20000"

/* Socket CMDS */
/* Offload connect CMDs */
#define SIMCOM_CMD_CONNECT			"AT+CAOPEN=0,%d,\"%s\",\"%s\",%d"
#define SIMCOM_CMD_CONNECT_MDM_CMD	\
		MODEM_CMD("+CAOPEN: ", on_cmd_offload_connect, 2U, "")

/* Offload send and recv CMDs */
#define SIMCOM_CMD_SOCKET_CLOSE			"AT+NSOCL=%d"
#define SIMCOM_CMD_DATA_SEND			"AT+CASEND=%d,%ld"
#define SIMCOM_CMD_DATA_RECV			"AT+CARECV=%d,%zd"
#define SIMCOM_CMD_MDM_RECV_CMD		\
		MODEM_CMD("+CARECV: ", on_cmd_recvfrom, 1U, "")

#define SIMCOM_CMD_PDP_ACTIVATE			"AT+CNACT=0,1"
#define CONNECT_ARGS				1
#define DATA_LEN_ARGS				0
#define DATA_ARGS				1

/* Unsoliciated CMDs */
#define SIMCOM_CMD_SOCKET_STAT_MDM_CMD	\
		MODEM_CMD("+CASTATE: ", on_urc_socket_status, 2U, ",")

#define SIMCOM_CMD_REPORT_DOWN_DATA_MDM_CMD \
		MODEM_CMD("+CADATAIND: ", on_urc_dataready, 1U, "")

#define SIMCOM_CMD_PDP_ACTIVE_CMD	\
		MODEM_CMD("+APP PDP: ", on_pdp_active, 2U, ",")

#define PDP_RESP_STR				strcmp(argv[1], "ACTIVE")

#endif
