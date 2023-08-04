/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/sntp.h>
#include <zephyr/posix/time.h>
#include <zephyr/zbus/zbus.h>
#include "ocpp.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#if defined(CONFIG_NET_DHCPV4)
static struct net_mgmt_event_callback dhcp_cb;
static void handler_cb(struct net_mgmt_event_callback *cb,
       uint32_t mgmt_event, struct net_if *iface)
{
       LOG_INF("net mgr cb");
       if(mgmt_event != NET_EVENT_IPV4_DHCP_BOUND) {
               return;
       }

       char buf[NET_IPV4_ADDR_LEN];

       LOG_INF("Your address: %s",
                       net_addr_ntop(AF_INET,
                               &iface->config.dhcpv4.requested_ip,
                               buf, sizeof(buf)));
       LOG_INF("Lease time: %u seconds",
                       iface->config.dhcpv4.lease_time);
       LOG_INF("Subnet: %s",
                       net_addr_ntop(AF_INET,
                               &iface->config.ip.ipv4->netmask,
                               buf, sizeof(buf)));
       LOG_INF("Router: %s",
                       net_addr_ntop(AF_INET,
                               &iface->config.ip.ipv4->gw,
                               buf, sizeof(buf)));
}

static int meta_dhcp_init(void)
{
       struct net_if *iface;

       net_mgmt_init_event_callback(&dhcp_cb, handler_cb,
                       NET_EVENT_IPV4_DHCP_BOUND);

       net_mgmt_add_event_callback(&dhcp_cb);

       iface = net_if_get_default();
       if (!iface) {
               LOG_ERR("wifi/eth interface not available");
               return -1;
       }

       net_dhcpv4_start(iface);

       k_msleep(2000);

       return 0;
}
#endif

#if defined(CONFIG_WIFI)
#include <esp_wifi.h>
static int meta_wifi_init(void)
{

       wifi_config_t wifi_config = {
               .sta = {
                       .ssid = "FRITZ!Box 7560 SU",
                       .password = "60359681488330619343",
               },
       };

       esp_err_t ret = esp_wifi_set_mode(ESP32_WIFI_MODE_STA);

       ret |= esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);

       ret |= esp_wifi_connect();
       if (ret != ESP_OK) {
               LOG_ERR("connection failed");
       }

       return 0;
}
#endif

#define NO_OF_CONN 4
K_KERNEL_STACK_ARRAY_DEFINE(cp_stk, NO_OF_CONN, 2 * 1024);
struct k_thread tinfo[NO_OF_CONN];
k_tid_t tid[NO_OF_CONN];
char idtag[NO_OF_CONN][25];

static int ocpp_get_time_from_sntp(void)
{
	struct sntp_ctx ctx;
	struct sntp_time sntp_time;
	struct sockaddr_in addr;
	struct timespec tv;
	int ret;

	/* ipv4 */
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(123);
	inet_pton(AF_INET, "162.159.200.1", &addr.sin_addr);

	ret = sntp_init(&ctx, (struct sockaddr *) &addr,
			sizeof(struct sockaddr_in));
	if (ret < 0) {
		LOG_ERR("Failed to init SNTP IPv4 ctx: %d", ret);
		return ret;
	}

	ret = sntp_query(&ctx, 60, &sntp_time);
	if (ret < 0) {
		LOG_ERR("SNTP IPv4 request failed: %d", ret);
		return ret;
	}

	printk("sntp succ since Epoch: %llu\n", sntp_time.seconds);
	tv.tv_sec = sntp_time.seconds;
	clock_settime(CLOCK_REALTIME, &tv);
	sntp_close(&ctx);
	return 0;
}

ZBUS_CHAN_DEFINE(ch_event, /* Name */
		 ocpp_io_value_t,  /* Message type */
		 NULL,          /* Validator */
		 NULL,   /* User data */
		 ZBUS_OBSERVERS_EMPTY, /* observers */
		 ZBUS_MSG_INIT(0) /* Initial value {0} */
);

ZBUS_SUBSCRIBER_DEFINE(cp_thread0, 5);
ZBUS_SUBSCRIBER_DEFINE(cp_thread1, 5);
ZBUS_SUBSCRIBER_DEFINE(cp_thread2, 5);
ZBUS_SUBSCRIBER_DEFINE(cp_thread3, 5);
struct zbus_observer *obs[] = {&cp_thread0, &cp_thread1, &cp_thread2, &cp_thread3};

static void cp_entry(void *p1, void *p2, void *p3);
static int user_notify_cb(ocpp_notify_reason_t reason,
			  ocpp_io_value_t *io,
			  void *user_data)
{
	static int wh = 6 + NO_OF_CONN;
	int idx;
	int i;
	//LOG_ERR("notify reson %u %x %x", reason, io, user_data);

	switch (reason) {
	case OCPP_USR_GET_METER_VALUE:
		if (OMM_ACTIVE_ENERGY_TO_EV == io->meter_val.mes) {
			snprintf(io->meter_val.val, CISTR50, "%u",
				 wh + io->meter_val.id_con);

			wh++;
			LOG_DBG("mtr reading val %s con %d",
					io->meter_val.val,
					io->meter_val.id_con);
			return 0;
		}
		break;

	case OCPP_USR_START_CHARGING:
		if (io->start_charge.id_con < 0) {
			for (i = 0; i < NO_OF_CONN; i++) {
				if (! tid[i]) {
					break;
				}
			}

			if (i >= NO_OF_CONN) {
				return -EBUSY;
			}
			idx = i;
		} else {
			idx = io->start_charge.id_con - 1;
		}

		if (! tid[idx]) {
			printk("start charging idtag %s connetor %d\n",
					idtag[idx], idx + 1);
			strncpy(idtag[idx], io->start_charge.idtag,
				sizeof(idtag[0]));
			tid[idx] = k_thread_create(&tinfo[idx], cp_stk[idx],
						sizeof(cp_stk[idx]), cp_entry,
						idx + 1, idtag[idx], obs[idx],
						7, 0, K_NO_WAIT);

			return 0;
		}
		break;

	case OCPP_USR_STOP_CHARGING:
		zbus_chan_pub(&ch_event, io, K_MSEC(100));
		return 0;

	case OCPP_USR_UNLOCK_CONNECTOR:
		printk("unlock connetor %d\n", io->unlock_con.id_con);
		return 0;
	}

	return -ENOTSUP;
}

static void cp_entry(void *p1, void *p2, void *p3)
{
	int ret;
	int stopWh;
	int idcon = (uint32_t)p1;
	char *idtag = (char *)p2;
	struct zbus_observer *obs = (struct zbus_observer *)p3;
	ocpp_session_handle_t sh = NULL;
	auth_status_t status = -1;

	ret = ocpp_session_open(&sh);
	printk("ocpp open ses idcon %d> res %d %x\n", idcon, ret, sh);
	if (ret)
		return ret;

	while(1) {
		k_sleep(K_SECONDS(10));
		ret = ocpp_authorize(sh,
				   idtag,
				   &status,
				   500);
		if (ret) {
			printk("ocpp auth %d> idcon %d status %d\n",
				 ret, idcon, status);
		} else {
			printk("ocpp auth %d> idcon %d status %d\n",
				 ret, idcon, status);
			break;
		}
	}

	if (status) {
		printk("ocpp start idcon %d> not authorized status %d\n",
				 idcon, status);
		return;
	}

	ret = ocpp_start_transaction(sh, idcon, idcon, 200);
	printk("ocpp start idcon %d> res %d\n", idcon, ret);
	if (!ret) {
		struct zbus_channel *chan;
		ocpp_io_value_t io;

		memset(&io, 0xff, sizeof(io));
		zbus_chan_add_obs(&ch_event, obs, K_SECONDS(1));
		do {
			zbus_sub_wait(obs, &chan, K_FOREVER);
			zbus_chan_read(chan, &io, K_SECONDS(1));

			if (io.stop_charge.id_con == idcon)
				break;

		} while(1);

		stopWh = 20;
	}

	ret = ocpp_stop_transaction(sh, idcon + stopWh, 200);
	printk("ocpp stop txn idcon %d> %d\n", idcon, ret);
	if (ret) {
		return;
	}

	k_sleep(K_SECONDS(1));
	ocpp_session_close(sh);
	tid[idcon - 1] = NULL;
	k_sleep(K_SECONDS(1));
        k_thread_abort(k_current_get());
}

int main(void)
{
	int ret;
	int i;

	ocpp_cp_info_t cpi = { "basic", "linumiz", .num_of_con = NO_OF_CONN};
	ocpp_cs_info_t csi =  {"192.168.1.3",
				"/steve/websocket/CentralSystemService/linumiz",
				8180,
				AF_INET};

	printk("Hello World! %s\n", CONFIG_BOARD);

#if defined(CONFIG_WIFI)
	meta_wifi_init();
#endif

#if defined(CONFIG_NET_DHCPV4)
	meta_dhcp_init();
#endif

	k_sleep(K_SECONDS(5));
	ocpp_get_time_from_sntp();
	printk("ocpp init %d\n", ret);
	ret = ocpp_init(&cpi,
			&csi,
			user_notify_cb,
			NULL);

	printk("ocpp init %d\n", ret);
	if (ret)
		return ret;

	for (i = 0; i < NO_OF_CONN; i++) {
		snprintf(idtag[i], sizeof(idtag[0]), "LinId%02d", i);
		tid[i] = k_thread_create(&tinfo[i], cp_stk[i],
					 sizeof(cp_stk[i]),
					 cp_entry, i + 1, idtag[i], obs[i],
					 7, 0, K_NO_WAIT);
	}

	k_sleep(K_SECONDS(60));

	for (i = 0; i < NO_OF_CONN; i++) {
		ocpp_io_value_t io = {0};
		io.stop_charge.id_con = i + 1;

		zbus_chan_pub(&ch_event, &io, K_MSEC(100));
		k_sleep(K_SECONDS(1));
	}

	k_sleep(K_SECONDS(1200));

	return 0;
}
