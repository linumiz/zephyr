/*
 * Copyright (C) 2021 metraTec GmbH
 * Copyright (C) 2023 Linumiz 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define  DT_DRV_COMPAT simcom

#include <zephyr/logging/log.h>
#include <zephyr/net/offloaded_netdev.h>
LOG_MODULE_REGISTER(modem_simcom, CONFIG_MODEM_LOG_LEVEL);

#if defined (CONFIG_MODEM_Y7080)
#include "simcom-y7080.h" 
#endif

#if defined (CONFIG_MODEM_SIM7080)
#include "simcom-sim7080.h"
#endif

#include <zephyr/drivers/modem/simcom-xxxx.h>
#include "simcom-xxxx.h"

static struct k_thread modem_rx_thread;
static struct k_work_q modem_workq;
static struct simcom_data mdata;
static struct modem_context mctx;
static const struct socket_op_vtable offload_socket_fd_op_vtable;

static struct zsock_addrinfo dns_result;
static struct sockaddr dns_result_addr;
static char dns_result_canonname[DNS_MAX_NAME_SIZE + 1];

static struct simcom_gnss_data gnss_data;

static const struct gpio_dt_spec pwr_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_power_gpios);

static K_KERNEL_STACK_DEFINE(modem_rx_stack, CONFIG_MODEM_SIMCOM_RX_STACK_SIZE);
static K_KERNEL_STACK_DEFINE(modem_workq_stack, CONFIG_MODEM_SIMCOM_RX_WORKQ_STACK_SIZE);
NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE, 0, NULL);

static void socket_close(struct modem_socket *sock);
const struct socket_dns_offload offload_dns_ops;

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

static inline uint8_t *modem_get_mac(const struct device *dev)
{
	struct simcom_data *data = dev->data;
	uint32_t hash_value;

	data->mac_addr[0] = 0x00;
	data->mac_addr[1] = 0x10;

	/* use IMEI for mac_addr */
	hash_value = hash32(mdata.mdm_imei, strlen(mdata.mdm_imei));

	UNALIGNED_PUT(hash_value, (uint32_t *)(data->mac_addr + 2));

	return data->mac_addr;
}

/**
 * Changes the operating state of the simcom.
 *
 * @param state new state.
 */
static void change_state(enum simcom_state state)
{
        LOG_DBG("Changing state to (%d)", state);
        mdata.state = state;
}

/**
 * Get the current operating state of the simcom.
 *
 * @return The current state.
 */
static enum simcom_state get_state(void)
{
        return mdata.state;
}

static int offload_socket(int family, int type, int proto);

/* Setup the Modem NET Interface. */
static void modem_net_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct simcom_data *data = dev->data;

	net_if_set_link_addr(iface, modem_get_mac(dev), sizeof(data->mac_addr), NET_LINK_ETHERNET);

	data->netif = iface;

	socket_offload_dns_register(&offload_dns_ops);

	net_if_socket_offload_set(iface, offload_socket);
}

/*
 * Unlock the tx ready semaphore if '> ' is received.
 */
MODEM_CMD_DIRECT_DEFINE(on_cmd_tx_ready)
{
	k_sem_give(&mdata.sem_tx_ready);
	return len;
}

MODEM_CMD_DEFINE(on_cmd_offload_connect)
{
	int result = atoi(argv[CONNECT_ARGS]);

	LOG_INF("offload_connect: %d", result);
	modem_cmd_handler_set_error(data, result);
	return 0;
}

/*
 * Connects an modem socket. Protocol can either be TCP or UDP.
 */
static int offload_connect(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	int ret;
	uint16_t dst_port;
	char ip_str[NET_IPV6_ADDR_LEN];
	struct modem_cmd cmd[] = { SIMCOM_CMD_CONNECT_MDM_CMD };
	char buf[256] = { 0 };

	/* Modem is not attached to the network. */
        if (get_state() != SIMCOM_STATE_NETWORK) {
                return -EAGAIN;
        }

	if (modem_socket_is_allocated(&mdata.socket_config, sock) == false) {
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
	} else if (addr->sa_family == AF_INET) {
		dst_port = ntohs(net_sin(addr)->sin_port);
	}

	ret = modem_context_sprint_ip_addr(addr, ip_str, sizeof(ip_str));
	if (ret != 0) {
		LOG_ERR("Failed to format IP!");
		errno = ENOMEM;
		return -1;
	}

#if defined CONFIG_MODEM_Y7080

	ret = modem_cmd_send_nolock(&mctx.iface, &mctx.cmd_handler, NULL, 0, SIMCOM_CMD_DATA_REPORTING,
							NULL, MDM_CONNECT_TIMEOUT);
	if(ret != 0) {
		LOG_ERR("Failed to set the down stream data reporting mode");
		return -ret;
	}

	ret = offload_connect_cmd(sock, addr, buf);
	if(ret != 0){
		LOG_ERR("Failed to build connect cmd");
		return ret;
	}

	ret = modem_cmd_send_nolock(&mctx.iface, &mctx.cmd_handler, NULL, 0, buf,
			     			NULL, MDM_CONNECT_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("%s ret: %d", buf, ret);
		goto error;
	}
	
	if(sock->type == SOCK_DGRAM)
		goto udp; 

#endif

	ret = snprintk(buf, sizeof(buf), SIMCOM_CMD_CONNECT, sock->id,
								#if defined CONFIG_MODEM_SIM7080
								(sock->type == SOCK_STREAM) ? "TCP" : "UDP",
								#endif
						       		 ip_str, dst_port);
	if (ret < 0) {
		LOG_ERR("Failed to build TCP connect command. ID: %d, FD: %d", sock->id, sock->sock_fd);
		errno = ENOMEM;
		return ret;
	}

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmd, ARRAY_SIZE(cmd), buf,
			     &mdata.sem_response, MDM_CONNECT_TIMEOUT);
	if(ret < 0){
		return ret;
	}

	ret =  modem_cmd_handler_get_error(&mdata.cmd_handler_data);
	if(ret != 0) {
		LOG_ERR("Failed to establish conenction");
		socket_close(sock);
		goto error;
	}
	
udp:
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
	char send_buf[64] = { 0 };
	char ctrlz = 0x1A;

	/* Modem is not attached to the network. */
        if (get_state() != SIMCOM_STATE_NETWORK) {
                LOG_ERR("Modem currently not attached to the network!");
                return -EAGAIN;
        }

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

	ret = snprintk(send_buf, sizeof(send_buf), SIMCOM_CMD_DATA_SEND, sock->sock_fd, (long)len);
	if (ret < 0) {
		LOG_ERR("Failed to build send command!!");
		errno = ENOMEM;
		return -1;
	}

	/* Make sure only one send can be done at a time. */
	k_sem_take(&mdata.cmd_handler_data.sem_tx_lock, K_FOREVER);
	k_sem_reset(&mdata.sem_tx_ready);

	/* Send */
	mdata.current_sock_written = len;
	ret = modem_cmd_send_nolock(&mctx.iface, &mctx.cmd_handler, NULL, 0U, send_buf, NULL,
				    K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to send data"); 
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

/*
 * Read data from a given socket.
 *
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

	ret = net_buf_linearize(sock_data->recv_buf, sock_data->recv_buf_len, data->rx_buf, 0,
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

/*
 * Handler for carecv response.
 */
MODEM_CMD_DEFINE(on_cmd_recvfrom)
{
	return sockread_common(mdata.current_sock_fd, data, atoi(argv[DATA_LEN_ARGS]), len);
}

/*
 * Read data from a given socket.
 */
static ssize_t offload_recvfrom(void *obj, void *buf, size_t max_len, int flags,
				struct sockaddr *src_addr, socklen_t *addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	char sendbuf[64];
	int ret, packet_size;
	struct socket_read_data sock_data;

	struct modem_cmd data_cmd[] = { SIMCOM_CMD_MDM_RECV_CMD };

	 /* Modem is not attached to the network. */
        if (get_state() != SIMCOM_STATE_NETWORK) {
                LOG_ERR("Modem currently not attached to the network!");
                return -EAGAIN;
        }

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
	snprintk(sendbuf, sizeof(sendbuf), SIMCOM_CMD_DATA_RECV, sock->id, max_len);

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
 * Sends messages to the modem.
 */
static ssize_t offload_sendmsg(void *obj, const struct msghdr *msg, int flags)
{
	struct modem_socket *sock = obj;
	ssize_t sent = 0;
	const char *buf;
	size_t len;
	int ret;

	 /* Modem is not attached to the network. */
        if (get_state() != SIMCOM_STATE_NETWORK) {
                LOG_ERR("Modem currently not attached to the network!");
                return -EAGAIN;
        }

	if (sock->type == SOCK_DGRAM) {
		/*
		 * Current implementation only handles single contiguous fragment at a time, so
		 * prevent sending multiple datagrams.
		 */
		if (msghdr_non_empty_iov_count(msg) > 1) {
			errno = EMSGSIZE;
			return -1;
		}
	}

	for (int i = 0; i < msg->msg_iovlen; i++) {
		buf = msg->msg_iov[i].iov_base;
		len = msg->msg_iov[i].iov_len;

		while (len > 0) {
			ret = offload_sendto(obj, buf, len, flags, msg->msg_name, msg->msg_namelen);
			if (ret < 0) {
				if (ret == -EAGAIN) {
					k_sleep(K_SECONDS(1));
				} else {
					return ret;
				}
			} else {
				sent += ret;
				buf += ret;
				len -= ret;
			}
		}
	}

	return sent;
}

/*
 * Closes a given socket.
 */
static void socket_close(struct modem_socket *sock)
{
	char buf[sizeof(SIMCOM_CMD_SOCKET_CLOSE + 1)];
	int ret;

	snprintk(buf, sizeof(buf), SIMCOM_CMD_SOCKET_CLOSE, sock->id);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, buf, &mdata.sem_response,
			     MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("%s ret: %d", buf, ret);
	}

	modem_socket_put(&mdata.socket_config, sock->sock_fd);
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

	 /* Modem is not attached to the network. */
        if (get_state() != SIMCOM_STATE_NETWORK) {
                LOG_ERR("Modem currently not attached to the network!");
                return -EAGAIN;
        }

	/* Make sure we assigned an id */
	if (modem_socket_is_allocated(&mdata.socket_config, sock) == false) {
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

	/* Modem is not attached to the network. */
	if (get_state() != SIMCOM_STATE_NETWORK) {
		LOG_ERR("Modem currently not attached to the network!");
		return -EAGAIN;
	}

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
	.sendmsg	= offload_sendmsg,
	.getsockopt	= NULL,
	.setsockopt	= NULL,
};

/*
 * Parses the dns response from the modem.
 */
MODEM_CMD_DEFINE(on_cmd_dns_resolve)
{
	int state;
	char ips[256];
	size_t out_len;
	int ret = -1;

	/* Offset to skip the leading " */
	out_len = net_buf_linearize(ips, sizeof(ips) - 1, data->rx_buf, 1, len);
	ips[out_len] = '\0';

	ret = net_addr_pton(dns_result.ai_family, ips,
		      &((struct sockaddr_in *)&dns_result_addr)->sin_addr);

	k_sem_give(&mdata.sem_dns);
	return ret;
}

/*
 * Perform a dns lookup.
 */
static int offload_getaddrinfo(const char *node, const char *service,
			       const struct zsock_addrinfo *hints, struct zsock_addrinfo **res)
{
	struct modem_cmd cmd[] = { SIMCOM_CMD_DNS_MDM_CMD };
	char sendbuf[256];
	uint32_t port = 0;
	int ret;

	/* Modem is not attached to the network. */
	if (get_state() != SIMCOM_STATE_NETWORK) {
		LOG_ERR("Modem currently not attached to the network!");
		return -EAGAIN;
	}

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

	if (port > 0U) {
		if (dns_result.ai_family == AF_INET) {
			net_sin(&dns_result_addr)->sin_port = htons(port);
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

	snprintk(sendbuf, sizeof(sendbuf), SIMCOM_CMD_DNS_RESOLVE, node);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmd, ARRAY_SIZE(cmd), sendbuf,
			     &mdata.sem_dns, MDM_DNS_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	*res = (struct zsock_addrinfo *)&dns_result;
	return 0;
}

/*
 * Free addrinfo structure.
 */
static void offload_freeaddrinfo(struct zsock_addrinfo *res)
{
	/* No need to free static memory. */
	res = NULL;
}

/*
 * DNS vtable.
 */
const struct socket_dns_offload offload_dns_ops = {
	.getaddrinfo = offload_getaddrinfo,
	.freeaddrinfo = offload_freeaddrinfo,
};

static struct offloaded_if_api api_funcs = {
	.iface_api.init = modem_net_iface_init,
};

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
 * Process all messages received from the modem.
 */
static void modem_rx(void)
{
	while (true) {
		/* Wait for incoming data */
		modem_iface_uart_rx_wait(&mctx.iface, K_FOREVER);

		modem_cmd_handler_process(&mctx.cmd_handler, &mctx.iface);
	}
}

MODEM_CMD_DEFINE(on_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_error)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_exterror)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/*
 * Handles pdp context urc.
 *
 * The urc has the form +APP PDP: <index>,<state>.
 * State can either be ACTIVE for activation or
 * DEACTIVE if disabled.
 */
MODEM_CMD_DEFINE(on_pdp_active)
{
	int result = 0;
	result = PDP_RESP_STR == 0;
	if(!result)
		net_if_carrier_on(mdata.netif);
	else
		net_if_carrier_off(mdata.netif);

	mdata.pdp_active = result;
	LOG_INF("PDP context: %u", mdata.pdp_active);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/*
 * Handles socket data notification.
 *
 * The sim modem sends and unsolicited msg
 * if data can be read from a socket.
 */
MODEM_CMD_DEFINE(on_urc_dataready)
{
	struct modem_socket *sock;
	int sock_fd;

	sock_fd = atoi(argv[0]);

	sock = modem_socket_from_fd(&mdata.socket_config, sock_fd);
	if (!sock) {
		return 0;
	}

	/* Modem does not tell packet size. Set dummy for receive. */
	modem_socket_packet_size_update(&mdata.socket_config, sock, 1);

	LOG_INF("Data available on socket: %d", sock_fd);
	modem_socket_data_ready(&mdata.socket_config, sock);

	return 0;
}

/*
 * Handles the socket state response.
 */
MODEM_CMD_DEFINE(on_urc_socket_status)
{
	struct modem_socket *sock;
	int sockfd, state;

	sockfd = atoi(argv[0]);
	state = atoi(argv[1]);

	sock = modem_socket_from_fd(&mdata.socket_config, sockfd);
	if (!sock) {
		return 0;
	}

	/* Only continue if socket was closed. */
	if (state != 0) {
		return 0;
	}

	LOG_INF("Socket close indication for socket: %d", sockfd);

	sock->is_connected = false;
	LOG_INF("Socket closed: %d", sockfd);

	return 0;
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

	/* Log the received information. */
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

	/* Log the received information. */
	LOG_INF("ICCID: %s", mdata.mdm_iccid);
	return 0;
}
#endif /* defined(CONFIG_MODEM_SIM_NUMBERS) */

/*
 * Parses the non urc C(E)REG and updates registration status.
 */
MODEM_CMD_DEFINE(on_cmd_cereg)
{
	mdata.mdm_registration = atoi(argv[1]);
	LOG_INF("CREG: %u", mdata.mdm_registration);
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

	if (rssi == 0) {
		mdata.mdm_rssi = -115;
	} else if (rssi == 1) {
		mdata.mdm_rssi = -111;
	} else if (rssi > 1 && rssi < 31) {
		mdata.mdm_rssi = -114 + 2 * rssi;
	} else if (rssi == 31) {
		mdata.mdm_rssi = -52;
	} else {
		mdata.mdm_rssi = -1000;
	}

	LOG_INF("RSSI: %d", mdata.mdm_rssi);
	return 0;
}

/*
 * Queries modem RSSI.
 *
 * If a work queue parameter is provided query work will
 * be scheduled. Otherwise rssi is queried once.
 */
static void modem_rssi_query_work(struct k_work *work)
{
	struct modem_cmd cmd[] = { MODEM_CMD("+CSQ: ", on_cmd_csq, 2U, ",") };
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
 * Possible responses by the simcom.
 */
static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK", on_cmd_ok, 0U, ""),
	MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
	MODEM_CMD("+CME ERROR: ", on_cmd_exterror, 1U, ""),
	MODEM_CMD_DIRECT(">", on_cmd_tx_ready),
};

/*
 * Possible unsolicited commands.
 */
static const struct modem_cmd unsolicited_cmds[] = {
	SIMCOM_CMD_PDP_ACTIVE_CMD,
#if defined CONFIG_MODEM_Y7080
	SIMCOM_CMD_REPORT_DOWN_DATA_MDM_CMD,
#endif
#if defined CONFIG_MODEM_SIM7080
	SIMCOM_CMD_SOCKET_STAT_MDM_CMD,
	SIMCOM_CMD_REPORT_DOWN_DATA_MDM_CMD,
#endif
};

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
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, SIMCOM_CMD_PDP_ACTIVATE, 
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

	net_if_carrier_on(mdata.netif);
	LOG_INF("Network active.");

error:
	return ret;
}

/*
 * Toggles the modems power gpio.
 */
static void modem_pwrkey(void)
{
	/* Power pin should be high for 1.5 seconds. */
	gpio_pin_set_dt(&pwr_gpio, 1);
	k_sleep(K_MSEC(1500));
	gpio_pin_set_dt(&pwr_gpio, 0);
	k_sleep(K_SECONDS(5));
}

/*
 * Commands to be sent at setup.
 */
static const struct setup_cmd setup_cmds[] = {
	SETUP_CMD_NOHANDLE("ATH"),
	SETUP_CMD("AT+CGMI", "", on_cmd_cgmi, 0U, ""),
	SETUP_CMD("AT+CGMM", "", on_cmd_cgmm, 0U, ""),
	SETUP_CMD("AT+CGMR", "", on_cmd_cgmr, 0U, ""),
	SETUP_CMD("AT+CGSN", "", on_cmd_cgsn, 0U, ""),
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	SETUP_CMD("AT+CIMI", "", on_cmd_cimi, 0U, ""),
	SETUP_CMD("AT+CCID", "", on_cmd_ccid, 0U, ""),
#endif /* defined(CONFIG_MODEM_SIM_NUMBERS) */
#if defined CONFIG_MODEM_SIM7080
	SIMCOM_MODE_SELECTION,
	SIMCOM_MODE_SELECTION_RAT,
#endif
	SIMCOM_SET_BANDS,
#if defined CONFIG_MODEM_SIMCOM_RAT_GSM
	SIMCOM_MODE_SELECTION_GSM,
#endif
	SETUP_CMD("AT+CPIN?", "+CPIN: ", on_cmd_cpin, 1U, ""),
};

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
		modem_pwrkey();
		/*
		 * The simcom has a autobaud function.
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
char *gnss_get_next_param(char *src, const char *delim, char **saveptr)
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

void gnss_skip_param(char **saveptr)
{
	gnss_get_next_param(NULL, ",", saveptr);
}

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
int gnss_split_on_dot(const char *src, int32_t *f1, int32_t *f2)
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

/*
 * Parses the Gnss response.
 *
 */
MODEM_CMD_DEFINE(on_cmd_gnssinfo)
{
	char gps_buf[MDM_GNSS_PARSER_MAX_LEN];
	size_t out_len = net_buf_linearize(gps_buf, sizeof(gps_buf) - 1, data->rx_buf, 0, len);

	gps_buf[out_len] = '\0';
	return parse_gnssinfo(gps_buf, &gnss_data);
}

int mdm_simcom_query_gnss(struct simcom_gnss_data *data)
{
	int ret;
	struct modem_cmd cmds[] = { SIMCOM_CMD_MDM_GPS_INFO };

	if (get_state() != SIMCOM_STATE_GNSS) {
                LOG_ERR("GNSS functionality is not enabled!!");
                return -1;
        }

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmds, ARRAY_SIZE(cmds), SIMCOM_CMD_GPS_INFO,
			     &mdata.sem_response, K_SECONDS(2));
	if (ret < 0) {
		return ret;
	}

	if (data) {
		memcpy(data, &gnss_data, sizeof(gnss_data));
	}

	memset(&gnss_data, 0, sizeof(gnss_data));

	return ret;
}

int mdm_simcom_start_gnss(void)
{
	int ret;
#if defined CONFIG_MODEM_SIM7080
	change_state(SIMCOM_STATE_INIT);
        k_work_cancel_delayable(&mdata.rssi_query_work);

        ret = modem_autobaud();
        if (ret < 0) {
                LOG_ERR("Failed to start modem!!");
                return -1;
        }
#endif
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, SIMCOM_CMD_GPS_ON,
			     &mdata.sem_response, K_SECONDS(2));
	if (ret < 0) {
		return -1;
	}

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, SIMCOM_CMD_GPS_START,
			     &mdata.sem_response, K_SECONDS(2));
	if (ret < 0) {
		return -1;
	}	

#if defined CONFIG_MODEM_SIM7080
	change_state(SIMCOM_STATE_GNSS);
#else
	change_state(SIMCOM_STATE_NETWORK_AND_GNSS);
#endif
	return 0;
}

int mdm_simcom_stop_gnss(void)
{

	return modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, SIMCOM_CMD_GPS_STOP, 
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

#if defined CONFIG_MODEM_SIMCOM_GNSS_ONLY
	LOG_INF("Starting Modem in GNSS Mode only");
	return 0;
#endif

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

	ret = modem_pdp_activate();
	if (ret < 0) {
		goto error;
	}

	k_work_reschedule_for_queue(&modem_workq, &mdata.rssi_query_work,
				    K_SECONDS(RSSI_TIMEOUT_SECS));

	change_state(SIMCOM_STATE_NETWORK);
error:
	return ret;
}

int mdm_simcom_start_network(void)
{
        change_state(SIMCOM_STATE_INIT);
        return modem_setup();
}

int mdm_simcom_power_on(void)
{
        return modem_autobaud();
}

int mdm_simcom_power_off(void)
{
        int tries = 5;
        int autobaud_tries;
        int ret = 0;

        k_work_cancel_delayable(&mdata.rssi_query_work);

        /* Check if module is already off. */
        ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, "AT", &mdata.sem_response,
                             K_MSEC(1000));
        if (ret < 0) {
                change_state(SIMCOM_STATE_OFF);
                return 0;
        }

        while (tries--) {
                modem_pwrkey();

                autobaud_tries = 5;

                while (autobaud_tries--) {
                        ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, "AT",
                                             &mdata.sem_response, K_MSEC(500));
                        if (ret == 0) {
                                break;
                        }
                }

                if (ret < 0) {
                        change_state(SIMCOM_STATE_OFF);
                        return 0;
                }
        }

        return -1;
}

const char *mdm_simcom_get_manufacturer(void)
{
	return mdata.mdm_manufacturer;
}

const char *mdm_simcom_get_model(void)
{
	return mdata.mdm_model;
}

const char *mdm_simcom_get_revision(void)
{
	return mdata.mdm_revision;
}

const char *mdm_simcom_get_imei(void)
{
	return mdata.mdm_imei;
}

/*
 * Initializes modem handlers and context.
 * After successful init this function calls
 * modem_setup.
 */
static int modem_init(const struct device *dev)
{
	int ret;

	ARG_UNUSED(dev);

	mdata.netif = net_if_get_default();
        if (mdata.netif == NULL) {
                return -EIO;
        }

        net_if_carrier_off(mdata.netif);

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
	ret = modem_socket_init(&mdata.socket_config, &mdata.sockets[0], ARRAY_SIZE(mdata.sockets),
                               MDM_BASE_SOCKET_NUM, true, &offload_socket_fd_op_vtable);
	if (ret < 0) {
		goto error;
	}

	/* Command handler. */
	const struct modem_cmd_handler_config cmd_handler_config = {
		.match_buf = &mdata.cmd_match_buf[0],
		.match_buf_len = sizeof(mdata.cmd_match_buf),
		.buf_pool = &mdm_recv_pool,
		.alloc_timeout = BUF_ALLOC_TIMEOUT,
		.eol = "\r\n",
		.user_data = NULL,
		.response_cmds = response_cmds,
		.response_cmds_len = ARRAY_SIZE(response_cmds),
		.unsol_cmds = unsolicited_cmds,
		.unsol_cmds_len = ARRAY_SIZE(unsolicited_cmds),
	};

	change_state(SIMCOM_STATE_INIT);

	ret = modem_cmd_handler_init(&mctx.cmd_handler, &mdata.cmd_handler_data, &cmd_handler_config);
	if (ret < 0) {
		goto error;
	}

	/* Uart handler. */
	const struct modem_iface_uart_config uart_config = {
		.rx_rb_buf = &mdata.iface_rb_buf[0],
		.rx_rb_buf_len = sizeof(mdata.iface_rb_buf),
		.dev = MDM_UART_DEV,
		.hw_flow_control = DT_PROP(MDM_UART_NODE, hw_flow_control),
	};

	ret = modem_iface_uart_init(&mctx.iface, &mdata.iface_data, &uart_config);
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

	ret = gpio_pin_configure_dt(&pwr_gpio, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "power");
		goto error;
	}

	mctx.driver_data = &mdata;

	memset(&gnss_data, 0, sizeof(gnss_data));

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
				  CONFIG_MODEM_SIMCOM_INIT_PRIORITY, &api_funcs,
				  MDM_MAX_DATA_LENGTH);

NET_SOCKET_OFFLOAD_REGISTER(simcom, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY,
			    AF_UNSPEC, offload_is_supported, offload_socket);
