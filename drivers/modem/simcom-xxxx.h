/*
 * Copyright (C) 2021 metraTec GmbH
 * Copyright (C) 2023 Linumiz 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SIMCOM_XXXX_H
#define SIMCOM_XXXX_H

#include <zephyr/kernel.h>
#include <ctype.h>
#include <inttypes.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <string.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_offload.h>
#include <zephyr/net/socket_offload.h>

#include "modem_context.h"
#include "modem_cmd_handler.h"
#include "modem_iface_uart.h"
#include "modem_socket.h"

#define MDM_UART_NODE DT_INST_BUS(0)
#define MDM_UART_DEV DEVICE_DT_GET(MDM_UART_NODE)
#define MDM_MAX_DATA_LENGTH 1024
#define MDM_RECV_BUF_SIZE 1024
#define MDM_MAX_SOCKETS 5
#define MDM_BASE_SOCKET_NUM 0
#define MDM_RECV_MAX_BUF 30
#define BUF_ALLOC_TIMEOUT K_SECONDS(1)
#define MDM_CMD_TIMEOUT K_SECONDS(10)
#define MDM_REGISTRATION_TIMEOUT K_SECONDS(180)
#define MDM_CONNECT_TIMEOUT K_SECONDS(90)
#define MDM_PDP_TIMEOUT K_SECONDS(120)
#define MDM_DNS_TIMEOUT K_SECONDS(210)
#define MDM_WAIT_FOR_RSSI_DELAY K_SECONDS(2)
#define MDM_WAIT_FOR_RSSI_COUNT 30
#define MDM_MAX_AUTOBAUD 5
#define MDM_MAX_CEREG_WAITS 40
#define MDM_MAX_CGATT_WAITS 40
#define MDM_BOOT_TRIES 4
#define MDM_GNSS_PARSER_MAX_LEN 128
#define MDM_APN CONFIG_MODEM_SIMCOM_APN
#define MDM_LTE_BANDS CONFIG_MODEM_SIMCOM_LTE_BANDS
#define RSSI_TIMEOUT_SECS 30

/*
 * Default length of modem data.
 */
#define MDM_MANUFACTURER_LENGTH 12
#define MDM_MODEL_LENGTH 16
#define MDM_REVISION_LENGTH 64
#define MDM_IMEI_LENGTH 16
#define MDM_IMSI_LENGTH 16
#define MDM_ICCID_LENGTH 32

#define SIMCOM_CMD_DEFINE(rsp, on_rsp_callback, args_no, delimiter) \
				 MODEM_CMD(rsp, on_rsp_callback, args_no, delimiter)

#define SIMCOM_SETUP_CMD_NO_HANDLE(send_cmd_) 		SETUP_CMD_NOHANDLE(send_cmd_)

enum simcom_state {
        SIMCOM_STATE_INIT = 0,
        SIMCOM_STATE_NETWORK,
        SIMCOM_STATE_GNSS,
	SIMCOM_STATE_NETWORK_AND_GNSS,
        SIMCOM_STATE_OFF,
};

typedef  int (*simcom_mdm_init)();

/*
 * Driver data.
 */
struct simcom_data {
	/*
	 * Network interface of the sim module.
	 */
	struct net_if *netif;
	uint8_t mac_addr[6];
	/*
	 * Uart interface of the modem.
	 */
	struct modem_iface_uart_data iface_data;
	uint8_t iface_rb_buf[MDM_MAX_DATA_LENGTH];
	/*
	 * Modem specific Init
	 */
	simcom_mdm_init init;
	/*
	 * Modem command handler.
	 */
	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_match_buf[MDM_RECV_BUF_SIZE + 1];
	/*
	 * Modem socket data.
	 */
	struct modem_socket_config socket_config;
	struct modem_socket sockets[MDM_MAX_SOCKETS];

	enum simcom_state state;
	/*
	 * RSSI work
	 */
	struct k_work_delayable rssi_query_work;
	/*
	 * Information over the modem.
	 */
	char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
	char mdm_model[MDM_MODEL_LENGTH];
	char mdm_revision[MDM_REVISION_LENGTH];
	char mdm_imei[MDM_IMEI_LENGTH];
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	char mdm_imsi[MDM_IMSI_LENGTH];
	char mdm_iccid[MDM_ICCID_LENGTH];
#endif /* #if defined(CONFIG_MODEM_SIM_NUMBERS) */
	int mdm_rssi;
	/*
	 * Current operating socket and statistics.
	 */
	int current_sock_fd;
	int current_sock_written;
	/*
	 * Network registration of the modem.
	 */
	uint8_t mdm_registration;
	/*
	 * Whether gprs is attached or detached.
	 */
	uint8_t mdm_cgatt;
	/*
	 * If the sim card is ready or not.
	 */
	bool cpin_ready;
	/*
	 * Flag if the PDP context is active.
	 */
	bool pdp_active;
	/*
	 * Semaphore(s).
	 */
	struct k_sem sem_response;
	struct k_sem sem_tx_ready;
	struct k_sem sem_dns;
};

/*
 * Socket read callback data.
 */
struct socket_read_data {
	char *recv_buf;
	size_t recv_buf_len;
	struct sockaddr *recv_addr;
	uint16_t recv_read_len;
};

void gnss_skip_param(char **saveptr);
char *gnss_get_next_param(char *src, const char *delim, char **saveptr);

#endif /* SIMCOM_XXXX_H */
