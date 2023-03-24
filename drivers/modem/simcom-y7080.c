/*
 * Copyright (C) 2023 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT simcom_y7080

#include <stdlib.h>
#include <stdio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_simcom_y7080, CONFIG_MODEM_LOG_LEVEL);

#include <zephyr/drivers/modem/simcom-y7080.h>
#include "simcom-y7080.h"

static struct k_thread modem_rx_thread;
static struct k_work_q modem_workq;
static struct y7080_data mdata;
static struct modem_context mctx;
static const struct socket_op_vtable offload_socket_fd_op_vtable;
const struct socket_dns_offload offload_dns_ops;

static struct zsock_addrinfo dns_result;
static struct sockaddr dns_result_addr;
static char dns_result_canonname[DNS_MAX_NAME_SIZE + 1];

static struct y7080_gnss_data gnss_data;

static K_KERNEL_STACK_DEFINE(modem_rx_stack, CONFIG_MODEM_SIMCOM_Y7080_RX_STACK_SIZE);
static K_KERNEL_STACK_DEFINE(modem_workq_stack, CONFIG_MODEM_SIMCOM_Y7080_RX_WORKQ_STACK_SIZE);
NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE, 0, NULL);

static int parse_cgnssinfo(char *gps_buf);

static inline uint32_t hash32(char *str, int len)
{
#define HASH_MULTIPLIER 37
	uint32_t h = 0;
	int i;

	for (i = 0; i < len; ++i) {
		h = (h * HASH_MULTIPLIER) + str[i];
	}

	return h;
}

/**
 * Performs the autobaud sequence until modem answers or limit is reached.
 *
 * @return On successful boot 0 is returned. Otherwise <0 is returned.
 */
static int modem_autobaud(void)
{
	int boot_tries = 0;
	int counter = 0;
	int ret;

	while (boot_tries++ <= MDM_BOOT_TRIES) {

		/*
		 * The Y7080 has a autobaud function.
		 * On startup multiple AT's are sent until
		 * a OK is received.
		 */
		counter = 0;
		while (counter < MDM_MAX_AUTOBAUD) {
			ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, "AT",
					     &mdata.sem_response, K_MSEC(500));

			/* OK was received. */
			if (ret == 0) {
				/* Disable echo */
				return modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U,
						      "ATE0", &mdata.sem_response, K_SECONDS(2));
			}

			counter++;
		}
	}

	return -1;
}

/*
 * Closes a given socket.
 */
static void socket_close(struct modem_socket *sock)
{
	char buf[sizeof("AT+NSOCL=##")];
	int ret;

	snprintk(buf, sizeof(buf), "AT+NSOCL=%d", sock->sock_fd);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, buf, &mdata.sem_response,
			     MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("%s ret: %d", buf, ret);
	}

	modem_socket_put(&mdata.socket_config, sock->sock_fd);
}

/*
 * Read manufacturer identification.
 */
MODEM_CMD_DEFINE(on_cmd_cgmi)
{
	size_t out_len = net_buf_linearize(
		mdata.mdm_manufacturer, sizeof(mdata.mdm_manufacturer) - 1, data->rx_buf, 0, len);
	mdata.mdm_manufacturer[out_len] = '\0';
	LOG_INF("Manufacturer: %s", mdata.mdm_manufacturer);
	return 0;
}

/*
 * Read model identification.
 */
MODEM_CMD_DEFINE(on_cmd_cgmm)
{
	size_t out_len = net_buf_linearize(mdata.mdm_model, sizeof(mdata.mdm_model) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_model[out_len] = '\0';
	LOG_INF("Model: %s", mdata.mdm_model);
	return 0;
}

/*
 * Read software release.
 *
 * Response will be in format RESPONSE: <revision>.
 */
MODEM_CMD_DEFINE(on_cmd_cgmr)
{
	size_t out_len;
	char *p;

	out_len = net_buf_linearize(mdata.mdm_revision, sizeof(mdata.mdm_revision) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_revision[out_len] = '\0';

	/* The module prepends a Revision: */
	p = strchr(mdata.mdm_revision, ':');
	if (p) {
		out_len = strlen(p + 1);
		memmove(mdata.mdm_revision, p + 1, out_len + 1);
	}

	LOG_INF("Revision: %s", mdata.mdm_revision);
	return 0;
}

/*
 * Read serial number identification.
 */
MODEM_CMD_DEFINE(on_cmd_cgsn)
{
	size_t out_len =
		net_buf_linearize(mdata.mdm_imei, sizeof(mdata.mdm_imei) - 1, data->rx_buf, 0, len);
	mdata.mdm_imei[out_len] = '\0';
	LOG_INF("IMEI: %s", mdata.mdm_imei);
	return 0;
}

#if defined(CONFIG_MODEM_SIM_NUMBERS)
/*
 * Read international mobile subscriber identity.
 */
MODEM_CMD_DEFINE(on_cmd_cimi)
{
	size_t out_len =
		net_buf_linearize(mdata.mdm_imsi, sizeof(mdata.mdm_imsi) - 1, data->rx_buf, 0, len);
	mdata.mdm_imsi[out_len] = '\0';

	LOG_INF("IMSI: %s", mdata.mdm_imsi);
	return 0;
}

/*
 * Read iccid.
 */
MODEM_CMD_DEFINE(on_cmd_ccid)
{
	size_t out_len = net_buf_linearize(mdata.mdm_iccid, sizeof(mdata.mdm_iccid) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_iccid[out_len] = '\0';

	LOG_INF("ICCID: %s", mdata.mdm_iccid);
	return 0;
}
#endif /* defined(CONFIG_MODEM_SIM_NUMBERS) */

MODEM_CMD_DEFINE(on_cmd_cereg)
{
	mdata.mdm_registration = atoi(argv[1]);
	LOG_INF("CEREG: %u", mdata.mdm_registration);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_cpin)
{
	mdata.cpin_ready = strcmp(argv[0], "READY") == 0;
	LOG_INF("CPIN: %d", mdata.cpin_ready);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_cgatt)
{
	mdata.mdm_cgatt = atoi(argv[0]);
	LOG_INF("CGATT: %d", mdata.mdm_cgatt);
	return 0;
}

/*
 * Handler for RSSI query.
 *
 * +CSQ: <rssi>,<ber>
 *  rssi: 0,-115dBm; 1,-111dBm; 2...30,-110...-54dBm; 31,-52dBm or greater.
 *        99, ukn
 *  ber: Not used.
 */
MODEM_CMD_DEFINE(on_cmd_csq)
{
	int rssi = atoi(argv[0]);

	if (rssi <= 0) {
		mdata.mdm_rssi = -113;
	} else if (rssi == 1) {
		mdata.mdm_rssi = -111;
	} else if (rssi > 1 && rssi < 31) {
		mdata.mdm_rssi = -114 + 2 * rssi;
	} else if (rssi == 31) {
		mdata.mdm_rssi = -51;
	} else {
		mdata.mdm_rssi = -1000;
	}

	LOG_INF("RSSI: %d", mdata.mdm_rssi);
	return 0;
}

/*
 * Commands to be sent at setup.
 */
static const struct setup_cmd setup_cmds[] = {
	SETUP_CMD("AT+CGMI", "", on_cmd_cgmi, 0U, ""),
	SETUP_CMD("AT+CGMM", "", on_cmd_cgmm, 0U, ""),
	SETUP_CMD("AT+CGMR", "", on_cmd_cgmr, 0U, ""),
	SETUP_CMD("AT+CGSN", "", on_cmd_cgsn, 0U, ""),
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	SETUP_CMD("AT+CIMI", "", on_cmd_cimi, 0U, ""),
	SETUP_CMD("AT+NCCID", "", on_cmd_ccid, 0U, ""),
#endif /* defined(CONFIG_MODEM_SIM_NUMBERS) */
	SETUP_CMD_NOHANDLE("AT+NBAND=" MDM_LTE_BANDS),
	SETUP_CMD("AT+CPIN?", "+CPIN:", on_cmd_cpin, 1U, ""),
};

static inline uint8_t *modem_get_mac(const struct device *dev)
{
	struct y7080_data *data = dev->data;
	uint32_t hash_value;

	data->mac_addr[0] = 0x00;
	data->mac_addr[1] = 0x10;

	hash_value = hash32(mdata.mdm_imei, strlen(mdata.mdm_imei));

	UNALIGNED_PUT(hash_value, (uint32_t *)(data->mac_addr + 2));

	return data->mac_addr;
}

/*
 * Queries modem RSSI.
 *
 * If a work queue parameter is provided query work will
 * be scheduled. Otherwise rssi is queried once.
 */
static void modem_rssi_query_work(struct k_work *work)
{
	struct modem_cmd cmd[] = { MODEM_CMD("+CSQ:", on_cmd_csq, 2U, ",") };
	static char *send_cmd = "AT+CSQ";
	int ret;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmd, ARRAY_SIZE(cmd), send_cmd,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+CSQ ret:%d", ret);
	}

	if (work) {
		k_work_reschedule_for_queue(&modem_workq, &mdata.rssi_query_work,
					    K_SECONDS(RSSI_TIMEOUT_SECS));
	}
}

/*
 * Process all messages received from the modem.
 */
static void modem_rx(void)
{
	while (true) {
		k_sem_take(&mdata.iface_data.rx_sem, K_FOREVER);

		mctx.cmd_handler.process(&mctx.cmd_handler, &mctx.iface);
	}
}

MODEM_CMD_DEFINE(on_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_exterror)
{
	LOG_INF("+CME ERROR = %s", argv[0]);
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/*
 * Activates the pdp context
 */
static int modem_pdp_activate(void)
{
	int counter;
	int ret = 0;
	const char *buf = "AT+CEREG?";
	struct modem_cmd cmds[] = { MODEM_CMD("+CEREG:", on_cmd_cereg, 2U, ",") };

	struct modem_cmd cgatt_cmd[] = { MODEM_CMD("+CGATT:", on_cmd_cgatt, 1U, "") };

	counter = 0;
	while (counter++ < MDM_MAX_CGATT_WAITS && mdata.mdm_cgatt != 1) {
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cgatt_cmd,
				     ARRAY_SIZE(cgatt_cmd), "AT+CGATT?", &mdata.sem_response,
				     MDM_CMD_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("Failed to query cgatt!!");
			return -1;
		}
		k_sleep(K_SECONDS(1));
	}

	if (counter >= MDM_MAX_CGATT_WAITS) {
		LOG_WRN("Network attach failed!!");
		return -1;
	}

	if (!mdata.cpin_ready || mdata.mdm_cgatt != 1) {
		LOG_ERR("Fatal: Modem is not attached to GPRS network!!");
		return -1;
	}

	LOG_INF("Waiting for network");

	/* Wait until the module is registered to the network.
	 * Registration will be set by urc.
	 */
	counter = 0;
	while (counter++ < MDM_MAX_CEREG_WAITS && mdata.mdm_registration != 1 &&
	       mdata.mdm_registration != 5) {
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmds, ARRAY_SIZE(cmds), buf,
				     &mdata.sem_response, MDM_CMD_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("Failed to query registration!!");
			return -1;
		}

		k_sleep(K_SECONDS(1));
	}

	if (counter >= MDM_MAX_CEREG_WAITS) {
		LOG_WRN("Network registration failed!");
		ret = -1;
		goto error;
	}

	/*
	 * Now activate the pdp context and wait for confirmation.
	 */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, "AT+CGACT=1,0",
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Could not activate PDP context.");
		goto error;
	}

	ret = k_sem_take(&mdata.sem_response, MDM_PDP_TIMEOUT);
	if (ret < 0 || mdata.pdp_active == false) {
		LOG_ERR("Failed to activate PDP context.");
		ret = -1;
		goto error;
	}

	LOG_INF("Network active...");

error:
	return ret;
}

MODEM_CMD_DEFINE(on_urc_pdp)
{
        mdata.pdp_active = strcmp(argv[2], "ACT") == 0;
        LOG_INF("PDP context: %u", mdata.pdp_active);
        k_sem_give(&mdata.sem_response);
        return 0;
}

/*
 * Unlock the tx ready semaphore if '> ' is received.
 */
MODEM_CMD_DIRECT_DEFINE(on_cmd_tx_ready)
{
	k_sem_give(&mdata.sem_tx_ready);
	return len;
}

/*
 * Possible responses by the simcom-Y7080.
 */
static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK", on_cmd_ok, 0U, ""),
	MODEM_CMD("+CME ERROR:", on_cmd_exterror, 1U, ""),
	MODEM_CMD_DIRECT(">", on_cmd_tx_ready),
};

const char *mdm_y7080_get_manufacturer(void)
{
	return mdata.mdm_manufacturer;
}

const char *mdm_y7080_get_model(void)
{
	return mdata.mdm_model;
}

const char *mdm_y7080_get_revision(void)
{
	return mdata.mdm_revision;
}

const char *mdm_y7080_get_imei(void)
{
	return mdata.mdm_imei;
}

/*
 * Handles socket data notification.
 *
 * The sim modem sends and unsolicited +NSONMI:<sock><len>
 * if data can be read from a socket.
 */
MODEM_CMD_DEFINE(on_urc_nsonmi)
{
	struct modem_socket *sock;
	int sock_fd;
	size_t data_len;

	sock_fd = atoi(argv[0]);
	data_len = atoi(argv[1]);

	sock = modem_socket_from_fd(&mdata.socket_config, sock_fd);
	if (!sock) {
		return 0;
	}

	/* Modem does not tell packet size. Set dummy for receive. */
	modem_socket_packet_size_update(&mdata.socket_config, sock, data_len);

	LOG_INF("Data available on socket: %d", sock_fd);
	modem_socket_data_ready(&mdata.socket_config, sock);

	return 0;
}

MODEM_CMD_DEFINE(on_urc_gnssinfo)
{
	int ret;
	char gps_data[256] = { 0 };
	size_t out_len = 0;

	out_len = net_buf_linearize(gps_data, sizeof(gps_data) - 1, data->rx_buf, 4, len);
	gps_data[out_len] = '\0';

	return parse_cgnssinfo(gps_data);
}

/*
 * Possible unsolicited commands.
 */
static const struct modem_cmd unsolicited_cmds[] = {
	MODEM_CMD("+CGEV:", on_urc_pdp, 3U, " "),
	MODEM_CMD("+NSONMI:", on_urc_nsonmi, 2U, ","),
	MODEM_CMD("+CGNSSINFO:", on_urc_gnssinfo, 0U, ","),
	/* FIX ME */ /* Implement Socket status*/
};

/*
 * Parses the dns response from the modem.
 *
 * Response on success:
 * +XDNS:<ip address>
 */
MODEM_CMD_DEFINE(on_cmd_xdns)
{
	char ip[NET_IPV6_ADDR_LEN];
	size_t outlen;

	outlen = net_buf_linearize(ip, sizeof(ip) - 1 , data->rx_buf, 0, len);
	ip[outlen] = '\0';

	if(dns_result.ai_family == AF_INET6){
		net_addr_pton(dns_result.ai_family, ip,
			 &((struct sockaddr_in6 *)&dns_result_addr)->sin6_addr);
	}
	else {
		net_addr_pton(dns_result.ai_family, ip,
			     &((struct sockaddr_in *)&dns_result_addr)->sin_addr);
	}

	k_sem_give(&mdata.sem_dns);

	return 0;
}

/*
 * Perform a dns lookup.
 */
static int offload_getaddrinfo(const char *node, const char *service,
			       const struct zsock_addrinfo *hints, struct zsock_addrinfo **res)
{
	struct modem_cmd cmd[] = { MODEM_CMD("+XDNS:", on_cmd_xdns, 1U, "") };
	char sendbuf[sizeof("AT+XDNS=\"\",##,#####") + 128];
	uint32_t port = 0;
	int ret;

	/* init result */
	(void)memset(&dns_result, 0, sizeof(dns_result));
	(void)memset(&dns_result_addr, 0, sizeof(dns_result_addr));

	/* Currently only support IPv4. */
	dns_result.ai_family = hints->ai_family;
	dns_result_addr.sa_family = hints->ai_family;
	dns_result.ai_addr = &dns_result_addr;
	dns_result.ai_addrlen = sizeof(dns_result_addr);
	dns_result.ai_canonname = dns_result_canonname;
	dns_result_canonname[0] = '\0';

	if (service) {
		port = atoi(service);
		if (port < 1 || port > USHRT_MAX) {
			return DNS_EAI_SERVICE;
		}
	}

	/* Check if node is an IP address */
	if (net_addr_pton(dns_result.ai_family, node,
			  &((struct sockaddr_in *)&dns_result_addr)->sin_addr) == 0) {
		*res = &dns_result;
		return 0;
	}

	/* user flagged node as numeric host, but we failed net_addr_pton */
	if (hints && hints->ai_flags & AI_NUMERICHOST) {
		return DNS_EAI_NONAME;
	}

	snprintk(sendbuf, sizeof(sendbuf), "AT+XDNS=%s", node);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmd, ARRAY_SIZE(cmd), sendbuf,
			     &mdata.sem_dns, MDM_DNS_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	*res = (struct zsock_addrinfo *)&dns_result;

	return 0;
}

static bool offload_is_supported(int family, int type, int proto)
{
	if (family != AF_INET &&
	    family != AF_INET6) {
		return false;
	}

	if (type != SOCK_DGRAM &&
	    type != SOCK_STREAM) {
		return false;
	}

	if (proto != IPPROTO_TCP &&
	    proto != IPPROTO_UDP) {
		return false;
	}

	return true;
}

static int offload_socket(int family, int type, int proto)
{
	int ret;

	ret = modem_socket_get(&mdata.socket_config, family, type, proto);
	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return ret;
}

/*
 * DNS vtable.
 */
const struct socket_dns_offload offload_dns_ops = {
	.getaddrinfo = offload_getaddrinfo,
	.freeaddrinfo = NULL,
};

/* Setup the Modem NET Interface. */
static void modem_net_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct y7080_data *data = dev->data;

	net_if_set_link_addr(iface, modem_get_mac(dev), sizeof(data->mac_addr), NET_LINK_ETHERNET);

	data->netif = iface;

	socket_offload_dns_register(&offload_dns_ops);

	net_if_socket_offload_set(iface, offload_socket);
}

static struct net_if_api api_funcs = {
	.init = modem_net_iface_init,
};

/**
 * Get the next parameter from the gnss phrase.
 *
 * @param src The source string supported on first call.
 * @param delim The delimiter of the parameter list.
 * @param saveptr Pointer for subsequent parses.
 * @return On success a pointer to the parameter. On failure
 *         or end of string NULL is returned.
 *
 * This function is used instead of strtok because strtok would
 * skip empty parameters, which is not desired. The modem may
 * omit parameters which could lead to a incorrect parse.
 */
static char *gnss_get_next_param(char *src, const char *delim, char **saveptr)
{
	char *start, *del;

	if (src) {
		start = src;
	} else {
		start = *saveptr;
	}

	/* Illegal start string. */
	if (!start) {
		return NULL;
	}

	/* End of string reached. */
	if (*start == '\0' || *start == '\r') {
		return NULL;
	}

	del = strstr(start, delim);
	if (!del) {
		return NULL;
	}

	*del = '\0';
	*saveptr = del + 1;

	if (del == start) {
		return NULL;
	}

	return start;
}

static void gnss_skip_param(char **saveptr)
{
	gnss_get_next_param(NULL, ",", saveptr);
}

#if 0
/**
 * Splits float parameters of the CGNSINF response on '.'
 *
 * @param src Null terminated string containing the float.
 * @param f1 Resulting number part of the float.
 * @param f2 Resulting fraction part of the float.
 * @return 0 if parsing was successful. Otherwise <0 is returned.
 *
 * If the number part of the float is negative f1 and f2 will be
 * negative too.
 */
static int gnss_split_on_dot(const char *src, int32_t *f1, int32_t *f2)
{
	char *dot = strchr(src, '.');

	if (!dot) {
		return -1;
	}

	*dot = '\0';

	*f1 = (int32_t)strtol(src, NULL, 10);
	*f2 = (int32_t)strtol(dot + 1, NULL, 10);

	if (*f1 < 0) {
		*f2 = -*f2;
	}

	return 0;
}
#endif

static float get_float_from_str(char *str)
{
	return atof(str);
}

static int get_int_from_str(char *str)
{
	return atoi(str);
}

static int parse_cgnssinfo(char *gps_buf)
{
	char *saveptr;
	int ret;
	int32_t number, fraction;

	char *lat = gnss_get_next_param(NULL, ",", &saveptr);
	if (lat == NULL) {
		goto error;
	}

	char *lat_dir = gnss_get_next_param(NULL, ",", &saveptr);
	if(lat_dir == NULL) {
		goto error;
	}

	char *lon = gnss_get_next_param(NULL, ",", &saveptr);
	if (lon == NULL) {
		goto error;
	}

	char *lon_dir = gnss_get_next_param(NULL, ",", &saveptr);
	if(lon_dir == NULL) {
		goto error;
	}

	char *date = gnss_get_next_param(NULL, ",", &saveptr);
	if(date == NULL){
		goto error;
	}

	char *utc_time = gnss_get_next_param(NULL, ",", &saveptr);
	if(utc_time == NULL){
		goto error;
	}

	char *alt = gnss_get_next_param(NULL, ",", &saveptr);
	if(alt == NULL){
		goto error;
	}

	char *speed = gnss_get_next_param(NULL, ",", &saveptr);
	if(speed == NULL){
		goto error;
	}

	char *course = gnss_get_next_param(NULL, ",", &saveptr);
	if(course == NULL){
		goto error;
	}

	gnss_skip_param(&saveptr);

	char *hdop = gnss_get_next_param(NULL, ",", &saveptr);
	if (hdop == NULL) {
		goto error;
	}

	gnss_data.lat = get_float_from_str(lat);
	gnss_data.lat_dir = lat_dir[0];
	gnss_data.lon = get_float_from_str(lon) ;
	gnss_data.lon_dir = lon_dir[0];
	if (alt) {
		gnss_data.alt =  get_float_from_str(alt);;
	} else {
		gnss_data.alt = 0;
	}

	strcpy(gnss_data.date, date);

	strcpy(gnss_data.utc, utc_time);

	if(speed)
		gnss_data.knots = get_float_from_str(speed);

	gnss_data.hdop = get_int_from_str(hdop);

	if(course)
		gnss_data.cog = get_int_from_str(course);

	return 0;
error:
	memset(&gnss_data, 0, sizeof(gnss_data));
	return -1;
}

int mdm_y7080_start_gnss(void)
{
	int ret = 0;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, "AT+CGNSSPWR=1",
			     &mdata.sem_response, K_SECONDS(2));
	if (ret < 0) {
		return ret;
	}

#if MODEM_SIMCOM_Y7080_GNSS_HOT_START
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, "AT+CGPSHOT=1",
			&mdata.sem_response, K_SECONDS(2));
	if(ret < 0){
		return ret;
	}

#elif MODEM_SIMCOM_Y7080_GNSS_WARM_START
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, "AT+CGPSWARM=1",
			&mdata.sem_response, K_SECONDS(2));
	if(ret < 0){
		return ret;
	}

#else
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, "AT+CGPSCOLD",
		&mdata.sem_response, K_SECONDS(2));
	if(ret < 0){
		return ret;
	}

#endif
	char buf[sizeof("AT+GNSSINFO=###")] = { 0 };

	snprintf(buf, sizeof(buf), "AT+CGNSSINFO=%u", CONFIG_SIMCOM_Y7080_GNSS_INFO_TIME);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, buf,
			&mdata.sem_response, K_SECONDS(2));
	if(ret < 0){
		return ret;
	}

	LOG_INF("SIMCOM Y7080 GNSS Started");

	return 0;
}

int mdm_y7080_stop_gnss(void)
{
	return modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, "AT+CGNSSPWR=0",
			&mdata.sem_response, K_SECONDS(2));

}

/*
 * Does the modem setup by starting it and
 * bringing the modem to a PDP active state.
 */
static int modem_setup(void)
{
	int ret = 0;
	int counter = 0;

	k_work_cancel_delayable(&mdata.rssi_query_work);

	ret = modem_autobaud();
	if (ret < 0) {
		LOG_ERR("Booting modem failed!!");
		goto error;
	}

	ret = modem_cmd_handler_setup_cmds(&mctx.iface, &mctx.cmd_handler, setup_cmds,
					   ARRAY_SIZE(setup_cmds), &mdata.sem_response,
					   MDM_REGISTRATION_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Failed to send init commands!");
		goto error;
	}

	k_sleep(K_SECONDS(3));

	/* Wait for acceptable rssi values. */
	modem_rssi_query_work(NULL);
	k_sleep(MDM_WAIT_FOR_RSSI_DELAY);

	counter = 0;
	while (counter++ < MDM_WAIT_FOR_RSSI_COUNT &&
	       (mdata.mdm_rssi >= 0 || mdata.mdm_rssi <= -1000)) {
		modem_rssi_query_work(NULL);
		k_sleep(MDM_WAIT_FOR_RSSI_DELAY);
	}

	if (mdata.mdm_rssi >= 0 || mdata.mdm_rssi <= -1000) {
		LOG_ERR("Network not reachable!!");
		ret = -ENETUNREACH;
		goto error;
	}
#if 0
	ret = modem_pdp_activate();
	if (ret < 0) {
		goto error;
	}
#endif
	k_work_reschedule_for_queue(&modem_workq, &mdata.rssi_query_work,
				    K_SECONDS(RSSI_TIMEOUT_SECS));

error:
	return ret;
}

MODEM_CMD_DEFINE(on_cmd_nsoco)
{
	int result = atoi(argv[0]);

	LOG_INF("+NSOCLI:%d", result);
	modem_cmd_handler_set_error(data, result);
	return 0;
}

static int modem_tcp_connect(struct modem_socket *sock, const struct sockaddr *addr)
{
	struct modem_cmd cmd[] = { MODEM_CMD("+NSOCLI:", on_cmd_nsoco, 1U, "") };
	char buf[sizeof("AT+NSOCO:#,xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx,#####")];
	char ip_str[NET_IPV6_ADDR_LEN];
	int ret;
	uint16_t dst_port;

	/* get the destination port */
	if (addr->sa_family == AF_INET6) {
		dst_port = ntohs(net_sin6(addr)->sin6_port);
	} else if (addr->sa_family == AF_INET) {
		dst_port = ntohs(net_sin(addr)->sin_port);
	}

	ret = modem_context_sprint_ip_addr(addr, ip_str, sizeof(ip_str));
	if (ret != 0) {
		LOG_ERR("Failed to format IP!");
		errno = ENOMEM;
		return ret;
	}

	ret = snprintk(buf, sizeof(buf), "AT+NSOCO=%d, %s, %d", sock->sock_fd,
								ip_str, dst_port);
	if (ret < 0) {
		LOG_ERR("Failed to build connect command. ID: %d, FD: %d", sock->id, sock->sock_fd);
		errno = ENOMEM;
		return ret;
	}

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmd, ARRAY_SIZE(cmd), buf,
			     &mdata.sem_response, MDM_CONNECT_TIMEOUT);
	if(ret < 0){
		return ret;
	}

	return modem_cmd_handler_get_error(&mdata.cmd_handler_data);
}

MODEM_CMD_DEFINE(on_cmd_nsocr)
{
	int result = atoi(argv[0]);

	LOG_INF("+NSOCR:%d", result);
	modem_cmd_handler_set_error(data, result);
	return 0;
}

/*
 * Connects an modem socket. Protocol can either be TCP or UDP.
 */
static int offload_connect(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	uint16_t dst_port = 0;
	char *protocol_type;
	uint8_t protocol;
	uint8_t receive_ctrl;
	char af_type[] = { 0 };
	struct modem_cmd cmd[] = { MODEM_CMD("+NSOCR:", on_cmd_nsocr, 1U, "") };
	char buf[sizeof("AT+NSOCR:#####,##,#####,##,#,########")];
	char ip_str[NET_IPV6_ADDR_LEN];
	int ret;

	/*FIXME check connection */

	if (sock->id < mdata.socket_config.base_socket_num - 1) {
		LOG_ERR("Invalid socket id %d from fd %d", sock->id, sock->sock_fd);
		errno = EINVAL;
		return -1;
	}

	if (sock->is_connected == true) {
		LOG_ERR("Socket is already connected! id: %d, fd: %d", sock->id, sock->sock_fd);
		errno = EISCONN;
		return -1;
	}

	/* get the destination port */
	if (addr->sa_family == AF_INET6) {
		dst_port = ntohs(net_sin6(addr)->sin6_port);
		strcpy(af_type, "AF_INET6");
	} else if (addr->sa_family == AF_INET) {
		dst_port = ntohs(net_sin(addr)->sin_port);
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
		errno = ENOMEM;
		return -1;
	}

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmd, ARRAY_SIZE(cmd), buf,
			     &mdata.sem_response, MDM_CONNECT_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("%s ret: %d", buf, ret);
		socket_close(sock);
		goto error;
	}

	ret = modem_cmd_handler_get_error(&mdata.cmd_handler_data);
	if (ret != 0) {
		LOG_ERR("Closing the socket!");
		socket_close(sock);
		goto error;
	}

	if(sock->type == SOCK_STREAM){
		ret = modem_tcp_connect(sock, addr);
		if(ret < 0) {
			LOG_ERR("TCP connection Failed");
			socket_close(sock);
			goto error;
		}
	}

	sock->is_connected = true;
	errno = 0;
	return 0;
error:
	errno = -ret;
	return -1;
}

/*
 * Send data over a given socket.
 *
 * First we signal the module that we want to send data over a socket.
 * This is done by sending AT+NSOSD=<sockfd>,<nbytes>\r\n.
 * If The module is ready to send data it will send back
 * an UNTERMINATED prompt '> '. After that data can be sent to the modem.
 * As terminating byte a STRG+Z (0x1A) is sent. The module will
 * then send a OK or ERROR.
 */
static ssize_t offload_sendto(void *obj, const void *buf, size_t len, int flags,
			      const struct sockaddr *dest_addr, socklen_t addrlen)
{
	int ret;
	struct modem_socket *sock = (struct modem_socket *)obj;
	char send_buf[sizeof("AT+NSOSD=#,####")] = { 0 };
	char ctrlz = 0x1A;

	/*FIXME Check connection */

	/* Do some sanity checks. */
	if (!buf || len == 0) {
		errno = EINVAL;
		return -1;
	}

	/* Socket has to be connected. */
	if (!sock->is_connected) {
		errno = ENOTCONN;
		return -1;
	}

	/* Only send up to MTU bytes. */
	if (len > MDM_MAX_DATA_LENGTH) {
		len = MDM_MAX_DATA_LENGTH;
	}

	if(sock->type == SOCK_STREAM){
		ret = snprintk(send_buf, sizeof(send_buf), "AT+NSOSD=%d, %ld", sock->sock_fd, (long)len);
		if (ret < 0) {
			LOG_ERR("Failed to build send command!!");
			errno = ENOMEM;
			return -1;
		}
	}
	else {
		ret = snprintf(send_buf, sizeof(send_buf), "AT+NSOSTF=%d, %ld", sock->sock_fd, (long)len);
		if(ret < 0) {
			LOG_ERR("Failed to build send command!!");
			errno = ENOMEM;
			return -1;
		}
	}

	/* Make sure only one send can be done at a time. */
	k_sem_take(&mdata.cmd_handler_data.sem_tx_lock, K_FOREVER);
	k_sem_reset(&mdata.sem_tx_ready);

	/* Send */
	mdata.current_sock_written = len;
	ret = modem_cmd_send_nolock(&mctx.iface, &mctx.cmd_handler, NULL, 0U, send_buf, NULL,
				    K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to send Data!!");
		goto exit;
	}

	/* Wait for '> ' */
	ret = k_sem_take(&mdata.sem_tx_ready, K_SECONDS(2));
	if (ret < 0) {
		LOG_ERR("Timeout while waiting for tx");
		goto exit;
	}

	/* Send data */
	mctx.iface.write(&mctx.iface, buf, len);
	mctx.iface.write(&mctx.iface, &ctrlz, 1);

	/* Wait for the OK */
	k_sem_reset(&mdata.sem_response);
	ret = k_sem_take(&mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Timeout waiting for OK");
	}

exit:
	k_sem_give(&mdata.cmd_handler_data.sem_tx_lock);
	/* Data was successfully sent */

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return mdata.current_sock_written;
}
#if 0
MODEM_CMD_DEFINE(on_cmd_nsorf)
{
	struct modem_socket *sock;
	int sock_fd;
	size_t data_len;

	sock_fd = atoi(argv[0]);
	data_len = atoi(argv[1]);

	sock = modem_socket_from_fd(&mdata.socket_config, sock_fd);
	if (!sock) {
		return -1;
	}

	/* Modem does not tell packet size. Set dummy for receive. */
	modem_socket_packet_size_update(&mdata.socket_config, sock, data_len);

	LOG_INF("Data available on socket: %d", sock_fd);
	modem_socket_data_ready(&mdata.socket_config, sock);

	return 0;
}
#endif

/*
 * Read data from a given socket.
 *
 * The response has the form +NSORF:<socket>,<ip_addr>,<port>,<length>,<data>,<remaining_len>
 */
static int sockread_common(int sockfd, struct modem_cmd_handler_data *data, int socket_data_length,
			   uint16_t len)
{
	struct modem_socket *sock;
	struct socket_read_data *sock_data;
	int ret, packet_size;

	if (!len) {
		LOG_ERR("Invalid length, aborting");
		return -EAGAIN;
	}

	if (!data->rx_buf) {
		LOG_ERR("Incorrect format! Ignoring data!");
		return -EINVAL;
	}

	if (socket_data_length <= 0) {
		LOG_ERR("Length error (%d)", socket_data_length);
		return -EAGAIN;
	}

	if (net_buf_frags_len(data->rx_buf) < socket_data_length) {
		LOG_DBG("Not enough data -- wait!");
		return -EAGAIN;
	}

	sock = modem_socket_from_fd(&mdata.socket_config, sockfd);
	if (!sock) {
		LOG_ERR("Socket not found! (%d)", sockfd);
		ret = -EINVAL;
		goto exit;
	}

	sock_data = (struct socket_read_data *)sock->data;
	if (!sock_data) {
		LOG_ERR("Socket data not found! (%d)", sockfd);
		ret = -EINVAL;
		goto exit;
	}

	ret = net_buf_linearize(sock_data->recv_buf, sock_data->recv_buf_len, data->rx_buf, 4,
				(uint16_t)socket_data_length);
	data->rx_buf = net_buf_skip(data->rx_buf, ret);
	sock_data->recv_read_len = ret;
	if (ret != socket_data_length) {
		LOG_ERR("Total copied data is different then received data!"
			" copied:%d vs. received:%d",
			ret, socket_data_length);
		ret = -EINVAL;
		goto exit;
	}

exit:
	/* Indication only sets length to a dummy value. */
	packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	modem_socket_packet_size_update(&mdata.socket_config, sock, -packet_size);
	return ret;
}

MODEM_CMD_DEFINE(on_cmd_nsorf)
{
	return sockread_common(mdata.current_sock_fd, data, atoi(argv[3]), len);
}

/*
 * Read data from a given socket.
 */
static ssize_t offload_recvfrom(void *obj, void *buf, size_t max_len, int flags,
				struct sockaddr *src_addr, socklen_t *addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	char sendbuf[sizeof("AT+NSORF=##,####")];
	int ret, packet_size;
	struct socket_read_data sock_data;

	struct modem_cmd data_cmd[] = { MODEM_CMD("+NSORF:", on_cmd_nsorf, 6U, ",") };

	/*FIXME Check connection */

	if (!buf || max_len == 0) {
		errno = EINVAL;
		return -1;
	}

	if (flags & ZSOCK_MSG_PEEK) {
		errno = ENOTSUP;
		return -1;
	}

	packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	if (!packet_size) {
		if (flags & ZSOCK_MSG_DONTWAIT) {
			errno = EAGAIN;
			return -1;
		}

		modem_socket_wait_data(&mdata.socket_config, sock);
		packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	}

	max_len = (max_len > MDM_MAX_DATA_LENGTH) ? MDM_MAX_DATA_LENGTH : max_len;
	snprintk(sendbuf, sizeof(sendbuf), "AT+NSORF=%d,%zd", sock->sock_fd, max_len);

	memset(&sock_data, 0, sizeof(sock_data));
	sock_data.recv_buf = buf;
	sock_data.recv_buf_len = max_len;
	sock_data.recv_addr = src_addr;
	sock->data = &sock_data;
	mdata.current_sock_fd = sock->sock_fd;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, data_cmd, ARRAY_SIZE(data_cmd),
			     sendbuf, &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		errno = -ret;
		ret = -1;
		goto exit;
	}

	/* HACK: use dst address as src */
	if (src_addr && addrlen) {
		*addrlen = sizeof(sock->dst);
		memcpy(src_addr, &sock->dst, *addrlen);
	}

	errno = 0;
	ret = sock_data.recv_read_len;

exit:
	/* clear socket data */
	mdata.current_sock_fd = -1;
	sock->data = NULL;
	return ret;
}

/*
 * Offloads read by reading from a given socket.
 */
static ssize_t offload_read(void *obj, void *buffer, size_t count)
{
	return offload_recvfrom(obj, buffer, count, 0, NULL, 0);
}

/*
 * Offloads write by writing to a given socket.
 */
static ssize_t offload_write(void *obj, const void *buffer, size_t count)
{
	return offload_sendto(obj, buffer, count, 0, NULL, 0);
}

/*
 * Offloads close by terminating the connection and freeing the socket.
 */
static int offload_close(void *obj)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	/* Make sure we assigned an id */
	if (sock->id < mdata.socket_config.base_socket_num) {
		return 0;
	}

	/* Close the socket only if it is connected. */
	if (sock->is_connected) {
		socket_close(sock);
	}

	return 0;
}

/*
 * Polls a given socket.
 */
static int offload_poll(struct zsock_pollfd *fds, int nfds, int msecs)
{
	int i;
	void *obj;

	/*FIXME Check connection */

	/* Only accept modem sockets. */
	for (i = 0; i < nfds; i++) {
		if (fds[i].fd < 0) {
			continue;
		}

		/* If vtable matches, then it's modem socket. */
		obj = z_get_fd_obj(fds[i].fd,
				   (const struct fd_op_vtable *)&offload_socket_fd_op_vtable,
				   EINVAL);
		if (obj == NULL) {
			return -1;
		}
	}

	return modem_socket_poll(&mdata.socket_config, fds, nfds, msecs);
}

/*
 * Offloads ioctl. Only supported ioctl is poll_offload.
 */
static int offload_ioctl(void *obj, unsigned int request, va_list args)
{
	switch (request) {
	case ZFD_IOCTL_POLL_PREPARE:
		return -EXDEV;

	case ZFD_IOCTL_POLL_UPDATE:
		return -EOPNOTSUPP;

	case ZFD_IOCTL_POLL_OFFLOAD: {
		/* Poll on the given socket. */
		struct zsock_pollfd *fds;
		int nfds, timeout;

		fds = va_arg(args, struct zsock_pollfd *);
		nfds = va_arg(args, int);
		timeout = va_arg(args, int);

		return offload_poll(fds, nfds, timeout);
	}

	default:
		errno = EINVAL;
		return -1;
	}
}

static const struct socket_op_vtable offload_socket_fd_op_vtable = {
	.fd_vtable = {
		.read	= offload_read,
		.write	= offload_write,
		.close	= offload_close,
		.ioctl	= offload_ioctl,
	},
	.bind		= NULL,
	.connect	= offload_connect,
	.sendto		= offload_sendto,
	.recvfrom	= offload_recvfrom,
	.listen		= NULL,
	.accept		= NULL,
	.sendmsg	= NULL,
	.getsockopt	= NULL,
	.setsockopt	= NULL,
};

/*
 * Initializes modem handlers and context.
 * After successful init this function calls
 * modem_setup.
 */
static int modem_init(const struct device *dev)
{
	int ret;

	ARG_UNUSED(dev);

	k_sem_init(&mdata.sem_response, 0, 1);
	k_sem_init(&mdata.sem_tx_ready, 0, 1);
	k_sem_init(&mdata.sem_dns, 0, 1);
	k_work_queue_start(&modem_workq, modem_workq_stack,
			   K_KERNEL_STACK_SIZEOF(modem_workq_stack), K_PRIO_COOP(7), NULL);

	/* Assume the modem is not registered to the network. */
	mdata.mdm_registration = 0;
	mdata.cpin_ready = false;
	mdata.pdp_active = false;

	/* Socket config. */
	mdata.socket_config.sockets = &mdata.sockets[0];
	mdata.socket_config.sockets_len = ARRAY_SIZE(mdata.sockets);
	mdata.socket_config.base_socket_num = MDM_BASE_SOCKET_NUM;
	ret = modem_socket_init(&mdata.socket_config, &offload_socket_fd_op_vtable);
	if (ret < 0) {
		goto error;
	}

	/* Command handler. */
	mdata.cmd_handler_data.cmds[CMD_RESP] = response_cmds;
	mdata.cmd_handler_data.cmds_len[CMD_RESP] = ARRAY_SIZE(response_cmds);
	mdata.cmd_handler_data.cmds[CMD_UNSOL] = unsolicited_cmds;
	mdata.cmd_handler_data.cmds_len[CMD_UNSOL] = ARRAY_SIZE(unsolicited_cmds);
	mdata.cmd_handler_data.match_buf = &mdata.cmd_match_buf[0];
	mdata.cmd_handler_data.match_buf_len = sizeof(mdata.cmd_match_buf);
	mdata.cmd_handler_data.buf_pool = &mdm_recv_pool;
	mdata.cmd_handler_data.alloc_timeout = BUF_ALLOC_TIMEOUT;
	mdata.cmd_handler_data.eol = "\r\n";
	ret = modem_cmd_handler_init(&mctx.cmd_handler, &mdata.cmd_handler_data);
	if (ret < 0) {
		goto error;
	}

	/* Uart handler. */
	mdata.iface_data.rx_rb_buf = &mdata.iface_rb_buf[0];
	mdata.iface_data.rx_rb_buf_len = sizeof(mdata.iface_rb_buf);
	ret = modem_iface_uart_init(&mctx.iface, &mdata.iface_data, MDM_UART_DEV);
	if (ret < 0) {
		goto error;
	}

	mdata.current_sock_fd = -1;
	mdata.current_sock_written = 0;

	/* Modem data storage. */
	mctx.data_manufacturer = mdata.mdm_manufacturer;
	mctx.data_model = mdata.mdm_model;
	mctx.data_revision = mdata.mdm_revision;
	mctx.data_imei = mdata.mdm_imei;
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	mctx.data_imsi = mdata.mdm_imsi;
	mctx.data_iccid = mdata.mdm_iccid;
#endif /* #if defined(CONFIG_MODEM_SIM_NUMBERS) */
	mctx.data_rssi = &mdata.mdm_rssi;

	mctx.driver_data = &mdata;

	ret = modem_context_register(&mctx);
	if (ret < 0) {
		LOG_ERR("Error registering modem context: %d", ret);
		goto error;
	}

	k_thread_create(&modem_rx_thread, modem_rx_stack, K_KERNEL_STACK_SIZEOF(modem_rx_stack),
			(k_thread_entry_t)modem_rx, NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	/* Init RSSI query */
	k_work_init_delayable(&mdata.rssi_query_work, modem_rssi_query_work);

	return modem_setup();
error:
	return ret;
}

/* Register device with the networking stack. */
NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, modem_init, NULL, &mdata, NULL,
				  CONFIG_MODEM_SIMCOM_Y7080_INIT_PRIORITY, &api_funcs,
				  MDM_MAX_DATA_LENGTH);

NET_SOCKET_OFFLOAD_REGISTER(simcom_y7080, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY,
			    AF_UNSPEC, offload_is_supported, offload_socket);

