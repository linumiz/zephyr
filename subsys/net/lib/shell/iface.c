/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <strings.h>
LOG_MODULE_DECLARE(net_shell);

#include <zephyr/sys/base64.h>

#if defined(CONFIG_NET_L2_ETHERNET)
#include <zephyr/net/ethernet.h>
#endif
#if defined(CONFIG_NET_L2_ETHERNET_MGMT)
#include <zephyr/net/ethernet_mgmt.h>
#endif
#if defined(CONFIG_NET_L2_VIRTUAL)
#include <zephyr/net/virtual.h>
#include <zephyr/net/virtual_mgmt.h>
#endif
#if defined(CONFIG_ETH_PHY_DRIVER)
#include <zephyr/net/phy.h>
#endif

#include "net_shell_private.h"

#define UNICAST_MASK GENMASK(7, 1)
#define LOCAL_BIT BIT(1)

#if defined(CONFIG_NET_L2_ETHERNET) && defined(CONFIG_NET_NATIVE)
struct ethernet_capabilities {
	enum ethernet_hw_caps capability;
	const char * const description;
};

#define EC(cap, desc) { .capability = cap, .description = desc }

static struct ethernet_capabilities eth_hw_caps[] = {
	EC(ETHERNET_HW_TX_CHKSUM_OFFLOAD, "TX checksum offload"),
	EC(ETHERNET_HW_RX_CHKSUM_OFFLOAD, "RX checksum offload"),
	EC(ETHERNET_HW_VLAN,              "Virtual LAN"),
	EC(ETHERNET_HW_VLAN_TAG_STRIP,    "VLAN Tag stripping"),
	EC(ETHERNET_LINK_10BASE,          "10 Mbits"),
	EC(ETHERNET_LINK_100BASE,         "100 Mbits"),
	EC(ETHERNET_LINK_1000BASE,        "1 Gbits"),
	EC(ETHERNET_LINK_2500BASE,        "2.5 Gbits"),
	EC(ETHERNET_LINK_5000BASE,        "5 Gbits"),
	EC(ETHERNET_PTP,                  "IEEE 802.1AS gPTP clock"),
	EC(ETHERNET_QAV,                  "IEEE 802.1Qav (credit shaping)"),
	EC(ETHERNET_QBV,                  "IEEE 802.1Qbv (scheduled traffic)"),
	EC(ETHERNET_QBU,                  "IEEE 802.1Qbu (frame preemption)"),
	EC(ETHERNET_TXTIME,               "TXTIME"),
	EC(ETHERNET_PROMISC_MODE,         "Promiscuous mode"),
	EC(ETHERNET_PRIORITY_QUEUES,      "Priority queues"),
	EC(ETHERNET_HW_FILTERING,         "MAC address filtering"),
	EC(ETHERNET_DSA_USER_PORT,        "DSA user port"),
	EC(ETHERNET_DSA_CONDUIT_PORT,     "DSA conduit port"),
	EC(ETHERNET_TXTIME,               "TXTIME supported"),
	EC(ETHERNET_TXINJECTION_MODE,     "TX-Injection supported"),
};

static void print_supported_ethernet_capabilities(
	const struct shell *sh, struct net_if *iface)
{
	enum ethernet_hw_caps caps = net_eth_get_hw_capabilities(iface);

	ARRAY_FOR_EACH(eth_hw_caps, i) {
		if (caps & eth_hw_caps[i].capability) {
			PR("\t%s\n", eth_hw_caps[i].description);
		}
	}
}
#endif /* CONFIG_NET_L2_ETHERNET */

#ifdef CONFIG_ETH_PHY_DRIVER
static void print_phy_link_state(const struct shell *sh, const struct device *phy_dev)
{
	struct phy_link_state link;
	int ret;

	ret = phy_get_link_state(phy_dev, &link);
	if (ret < 0) {
		PR_ERROR("Failed to get link state (%d)\n", ret);
		return;
	}

	PR("Ethernet link speed: %s ", PHY_LINK_IS_SPEED_1000M(link.speed)  ? "1 Gbits"
				       : PHY_LINK_IS_SPEED_100M(link.speed) ? "100 Mbits"
									    : "10 Mbits");

	PR("%s-duplex\n", PHY_LINK_IS_FULL_DUPLEX(link.speed) ? "full" : "half");
}
#endif

static const char *iface_flags2str(struct net_if *iface)
{
	static char str[sizeof("POINTOPOINT") + sizeof("PROMISC") +
			sizeof("NO_AUTO_START") + sizeof("SUSPENDED") +
			sizeof("MCAST_FORWARD") + sizeof("IPv4") +
			sizeof("IPv6") + sizeof("NO_ND") + sizeof("NO_MLD")];
	int pos = 0;

	if (net_if_flag_is_set(iface, NET_IF_POINTOPOINT)) {
		pos += snprintk(str + pos, sizeof(str) - pos,
				"POINTOPOINT,");
	}

	if (net_if_flag_is_set(iface, NET_IF_PROMISC)) {
		pos += snprintk(str + pos, sizeof(str) - pos,
				"PROMISC,");
	}

	if (net_if_flag_is_set(iface, NET_IF_NO_AUTO_START)) {
		pos += snprintk(str + pos, sizeof(str) - pos,
				"NO_AUTO_START,");
	} else {
		pos += snprintk(str + pos, sizeof(str) - pos,
				"AUTO_START,");
	}

	if (net_if_flag_is_set(iface, NET_IF_FORWARD_MULTICASTS)) {
		pos += snprintk(str + pos, sizeof(str) - pos,
				"MCAST_FORWARD,");
	}

	if (net_if_flag_is_set(iface, NET_IF_IPV4)) {
		pos += snprintk(str + pos, sizeof(str) - pos,
				"IPv4,");
	}

	if (net_if_flag_is_set(iface, NET_IF_IPV6)) {
		pos += snprintk(str + pos, sizeof(str) - pos,
				"IPv6,");
	}

	if (net_if_flag_is_set(iface, NET_IF_IPV6_NO_ND)) {
		pos += snprintk(str + pos, sizeof(str) - pos,
				"NO_ND,");
	}

	if (net_if_flag_is_set(iface, NET_IF_IPV6_NO_MLD)) {
		pos += snprintk(str + pos, sizeof(str) - pos,
				"NO_MLD,");
	}

	/* get rid of last ',' character */
	str[pos - 1] = '\0';

	return str;
}

static void iface_cb(struct net_if *iface, void *user_data)
{
	struct net_shell_user_data *data = user_data;
	const struct shell *sh = data->sh;
	int ret;

	ARG_UNUSED(ret); /* could be unused depending on config */

#if defined(CONFIG_NET_NATIVE_IPV6)
	struct net_if_ipv6_prefix *prefix;
	struct net_if_router *router;
#endif
#if defined(CONFIG_NET_IPV6)
	struct net_if_ipv6 *ipv6;
#endif
#if defined(CONFIG_NET_IPV4)
	struct net_if_ipv4 *ipv4;
#endif
#if defined(CONFIG_NET_IP)
	struct net_if_addr *unicast;
	struct net_if_mcast_addr *mcast;
#endif
#if defined(CONFIG_NET_L2_ETHERNET_MGMT)
	struct ethernet_req_params params;
#endif
	const char *extra;
#if defined(CONFIG_NET_IP) || defined(CONFIG_NET_L2_ETHERNET_MGMT)
	int count;
#endif

	if (data->user_data && data->user_data != iface) {
		return;
	}

#if defined(CONFIG_NET_INTERFACE_NAME)
	char ifname[CONFIG_NET_INTERFACE_NAME_LEN + 1] = { 0 };
	int ret_name;

	ret_name = net_if_get_name(iface, ifname, sizeof(ifname) - 1);
	if (ret_name < 1 || ifname[0] == '\0') {
		snprintk(ifname, sizeof(ifname), "?");
	}

	PR("\nInterface %s (%p) (%s) [%d]\n", ifname, iface, iface2str(iface, &extra),
	   net_if_get_by_iface(iface));
#else
	PR("\nInterface %p (%s) [%d]\n", iface, iface2str(iface, &extra),
	   net_if_get_by_iface(iface));
#endif
	PR("===========================%s\n", extra);

	if (!net_if_is_up(iface)) {
		PR_INFO("Interface is down.\n");

		/* Show detailed information only when user asks information
		 * about one specific network interface.
		 */
		if (data->user_data == NULL) {
			return;
		}
	}

#ifdef CONFIG_NET_POWER_MANAGEMENT
	if (net_if_is_suspended(iface)) {
		PR_INFO("Interface is suspended, thus not able to tx/rx.\n");
	}
#endif

#if defined(CONFIG_NET_L2_VIRTUAL)
	if (!sys_slist_is_empty(&iface->config.virtual_interfaces)) {
		struct virtual_interface_context *ctx, *tmp;

		PR("Virtual interfaces attached to this : ");
		SYS_SLIST_FOR_EACH_CONTAINER_SAFE(
					&iface->config.virtual_interfaces,
					ctx, tmp, node) {
			if (ctx->virtual_iface == iface) {
				continue;
			}

			PR("%d ", net_if_get_by_iface(ctx->virtual_iface));
		}

		PR("\n");
	}

	if (net_if_l2(iface) == &NET_L2_GET_NAME(VIRTUAL)) {
		struct net_if *orig_iface;
		char *name, buf[CONFIG_NET_L2_VIRTUAL_MAX_NAME_LEN];

		name = net_virtual_get_name(iface, buf, sizeof(buf));
		if (!(name && name[0])) {
			name = "<unknown>";
		}

		PR("Virtual name : %s\n", name);

		orig_iface = net_virtual_get_iface(iface);
		if (orig_iface == NULL) {
			PR("No attached network interface.\n");
		} else {
			PR("Attached  : %d (%s / %p)\n",
			   net_if_get_by_iface(orig_iface),
			   iface2str(orig_iface, NULL),
			   orig_iface);
		}
	}

	if (IS_ENABLED(CONFIG_NET_VPN) &&
	    net_if_l2(iface) == &NET_L2_GET_NAME(VIRTUAL)) {
		if (net_virtual_get_iface_capabilities(iface) & VIRTUAL_INTERFACE_VPN) {
			struct virtual_interface_req_params vparams = { 0 };
			char public_key[NET_VIRTUAL_MAX_PUBLIC_KEY_LEN * 2];
			size_t olen;

			ret = net_mgmt(NET_REQUEST_VIRTUAL_INTERFACE_GET_PUBLIC_KEY,
				       iface, &vparams, sizeof(vparams));
			if (ret < 0) {
				PR_WARNING("Cannot get VPN public key (%d)\n", ret);
			} else {
				bool all_zeros = true;

				for (int i = 0;
				     all_zeros && i < NET_VIRTUAL_MAX_PUBLIC_KEY_LEN; i++) {
					all_zeros = (vparams.public_key.data[i] == 0);
				}

				if (all_zeros) {
					PR("Public key: <not set>\n");
				} else {
					(void)base64_encode(public_key, sizeof(public_key),
							    &olen, vparams.public_key.data,
							    vparams.public_key.len);

					PR("Public key: %s\n", public_key);
				}
			}
		}
	}
#endif /* CONFIG_NET_L2_VIRTUAL */

	net_if_lock(iface);
	if (net_if_get_link_addr(iface)->len > 0) {
		PR("Link addr : %s\n",
		   net_sprint_ll_addr(net_if_get_link_addr(iface)->addr,
				      net_if_get_link_addr(iface)->len));
	}
	net_if_unlock(iface);

	PR("MTU       : %d\n", net_if_get_mtu(iface));
	PR("Flags     : %s\n", iface_flags2str(iface));
	PR("Device    : %s (%p)\n",
	   net_if_get_device(iface) ? net_if_get_device(iface)->name : "<?>",
	   net_if_get_device(iface));

	PR("Status    : oper=%s, admin=%s, carrier=%s\n",
	   net_if_oper_state2str(net_if_oper_state(iface)),
	   net_if_is_admin_up(iface) ? "UP" : "DOWN",
	   net_if_is_carrier_ok(iface) ? "ON" : "OFF");

#if defined(CONFIG_NET_IF_LOG_LEVEL_DBG)
	/* Print low level details only if debug is enabled */
	if (IS_ENABLED(CONFIG_NET_IPV4) && net_if_flag_is_set(iface, NET_IF_IPV4)) {
		PR("IPv4 TTL             : %d\n", net_if_ipv4_get_ttl(iface));
		PR("IPv4 mcast TTL       : %d\n", net_if_ipv4_get_mcast_ttl(iface));
	}

	if (IS_ENABLED(CONFIG_NET_IPV6) && net_if_flag_is_set(iface, NET_IF_IPV6)) {
		PR("IPv6 hop limit       : %d\n", net_if_ipv6_get_hop_limit(iface));
		PR("IPv6 mcast hop limit : %d\n",
		   net_if_ipv6_get_mcast_hop_limit(iface));
	}
#endif /* CONFIG_NET_IF_LOG_LEVEL_DBG */

#if defined(CONFIG_NET_L2_ETHERNET_MGMT)
	if (net_if_l2(iface) == &NET_L2_GET_NAME(ETHERNET)) {
		count = 0;
		ret = net_mgmt(NET_REQUEST_ETHERNET_GET_PRIORITY_QUEUES_NUM,
				iface, &params,
				sizeof(struct ethernet_req_params));

		if (!ret && params.priority_queues_num) {
			count = params.priority_queues_num;
			PR("Priority queues:\n");
			for (int i = 0; i < count; ++i) {
				params.qav_param.queue_id = i;
				params.qav_param.type =
					ETHERNET_QAV_PARAM_TYPE_STATUS;
				ret = net_mgmt(
					NET_REQUEST_ETHERNET_GET_QAV_PARAM,
					iface, &params,
					sizeof(struct ethernet_req_params));

				PR("\t%d: Qav ", i);
				if (ret) {
					PR("not supported\n");
				} else {
					PR("%s\n",
						params.qav_param.enabled ?
						"enabled" :
						"disabled");
				}
			}
		}
	}
#endif

#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	PR("Promiscuous mode : %s\n",
	   net_if_is_promisc(iface) ? "enabled" : "disabled");
#endif

#if defined(CONFIG_NET_VLAN)
	if (net_if_l2(iface) == &NET_L2_GET_NAME(VIRTUAL)) {
		if (net_virtual_get_iface_capabilities(iface) & VIRTUAL_INTERFACE_VLAN) {
			uint16_t tag;

			tag = net_eth_get_vlan_tag(iface);
			if (tag == NET_VLAN_TAG_UNSPEC) {
				PR("VLAN not configured\n");
			} else {
				PR("VLAN tag  : %d (0x%03x)\n", tag, tag);
			}
		}
	}
#endif

#ifdef CONFIG_NET_L2_ETHERNET
	if (net_if_l2(iface) == &NET_L2_GET_NAME(ETHERNET)) {
		PR("Ethernet capabilities supported:\n");
		print_supported_ethernet_capabilities(sh, iface);

#ifdef CONFIG_ETH_PHY_DRIVER
		const struct device *phy_dev = net_eth_get_phy(iface);

		PR("Ethernet PHY device: %s (%p)\n", (phy_dev != NULL) ? phy_dev->name : "<none>",
		   phy_dev);
		if (phy_dev != NULL) {
			print_phy_link_state(sh, phy_dev);
		}
#endif /* CONFIG_ETH_PHY_DRIVER */
	}
#endif /* CONFIG_NET_L2_ETHERNET */

#if defined(CONFIG_NET_IPV6)
	count = 0;
	ipv6 = iface->config.ip.ipv6;

	if (!net_if_flag_is_set(iface, NET_IF_IPV6) || ipv6 == NULL) {
		PR("%s not %s for this interface.\n", "IPv6", "enabled");
		ipv6 = NULL;
		goto skip_ipv6;
	}

	PR("IPv6 unicast addresses (max %d):\n", NET_IF_MAX_IPV6_ADDR);
	ARRAY_FOR_EACH(ipv6->unicast, i) {
		unicast = &ipv6->unicast[i];

		if (!unicast->is_used) {
			continue;
		}

		PR("\t%s %s %s%s%s%s\n",
		   net_sprint_ipv6_addr(&unicast->address.in6_addr),
		   addrtype2str(unicast->addr_type),
		   addrstate2str(unicast->addr_state),
		   unicast->is_infinite ? " infinite" : "",
		   unicast->is_mesh_local ? " meshlocal" : "",
		   unicast->is_temporary ? " temporary" : "");
		count++;
	}

	if (count == 0) {
		PR("\t<none>\n");
	}

	count = 0;

	PR("IPv6 multicast addresses (max %d):\n", NET_IF_MAX_IPV6_MADDR);
	ARRAY_FOR_EACH(ipv6->mcast, i) {
		mcast = &ipv6->mcast[i];

		if (!mcast->is_used) {
			continue;
		}

		PR("\t%s%s\n", net_sprint_ipv6_addr(&mcast->address.in6_addr),
		   net_if_ipv6_maddr_is_joined(mcast) ? "" : "  <not joined>");

		count++;
	}

	if (count == 0) {
		PR("\t<none>\n");
	}

#if defined(CONFIG_NET_NATIVE_IPV6)
	count = 0;

	PR("IPv6 prefixes (max %d):\n", NET_IF_MAX_IPV6_PREFIX);
	ARRAY_FOR_EACH(ipv6->prefix, i) {
		prefix = &ipv6->prefix[i];

		if (!prefix->is_used) {
			continue;
		}

		PR("\t%s/%d%s\n",
		   net_sprint_ipv6_addr(&prefix->prefix),
		   prefix->len, prefix->is_infinite ? " infinite" : "");

		count++;
	}

	if (count == 0) {
		PR("\t<none>\n");
	}

	router = net_if_ipv6_router_find_default(iface, NULL);
	if (router) {
		PR("IPv6 default router :\n");
		PR("\t%s%s\n",
		   net_sprint_ipv6_addr(&router->address.in6_addr),
		   router->is_infinite ? " infinite" : "");
	}
#endif /* CONFIG_NET_NATIVE_IPV6 */

skip_ipv6:

#if defined(CONFIG_NET_IPV6_PE)
	PR("IPv6 privacy extension   : %s (preferring %s addresses)\n",
	   iface->pe_enabled ? "enabled" : "disabled",
	   iface->pe_prefer_public ? "public" : "temporary");
#endif

	if (ipv6) {
		PR("IPv6 hop limit           : %d\n",
		   ipv6->hop_limit);
		PR("IPv6 base reachable time : %d\n",
		   ipv6->base_reachable_time);
		PR("IPv6 reachable time      : %d\n",
		   ipv6->reachable_time);
		PR("IPv6 retransmit timer    : %d\n",
		   ipv6->retrans_timer);
	}

#if defined(CONFIG_NET_DHCPV6)
	if (net_if_flag_is_set(iface, NET_IF_IPV6)) {
		if (iface->config.dhcpv6.state != NET_DHCPV6_DISABLED) {
			PR("DHCPv6 renewal time (T1) : %llu ms\n",
			   iface->config.dhcpv6.t1);
			PR("DHCPv6 rebind time (T2)  : %llu ms\n",
			   iface->config.dhcpv6.t2);
			PR("DHCPv6 expire time       : %llu ms\n",
			   iface->config.dhcpv6.expire);
			if (iface->config.dhcpv6.params.request_addr) {
				PR("DHCPv6 address           : %s\n",
				   net_sprint_ipv6_addr(&iface->config.dhcpv6.addr));
			}

			if (iface->config.dhcpv6.params.request_prefix) {
				PR("DHCPv6 prefix            : %s\n",
				   net_sprint_ipv6_addr(&iface->config.dhcpv6.prefix));
			}
		}

		PR("DHCPv6 state             : %s\n",
		   net_dhcpv6_state_name(iface->config.dhcpv6.state));
	}
#endif /* CONFIG_NET_DHCPV6 */
#endif /* CONFIG_NET_IPV6 */

#if defined(CONFIG_NET_IPV4)
	/* No need to print IPv4 information for interface that does not
	 * support that protocol.
	 */
	if (
#if defined(CONFIG_NET_L2_IEEE802154)
		(net_if_l2(iface) == &NET_L2_GET_NAME(IEEE802154)) ||
#endif
		 0) {
		PR_WARNING("%s not %s for this interface.\n", "IPv4",
			   "supported");
		return;
	}

	count = 0;
	ipv4 = iface->config.ip.ipv4;

	if (!net_if_flag_is_set(iface, NET_IF_IPV4) || ipv4 == NULL) {
		PR("%s not %s for this interface.\n", "IPv4", "enabled");
		ipv4 = NULL;
		goto skip_ipv4;
	}

	PR("IPv4 unicast addresses (max %d):\n", NET_IF_MAX_IPV4_ADDR);
	ARRAY_FOR_EACH(ipv4->unicast, i) {
		unicast = &ipv4->unicast[i].ipv4;

		if (!unicast->is_used) {
			continue;
		}

		PR("\t%s/%s %s %s%s\n",
		   net_sprint_ipv4_addr(&unicast->address.in_addr),
		   net_sprint_ipv4_addr(&ipv4->unicast[i].netmask),

		   addrtype2str(unicast->addr_type),
		   addrstate2str(unicast->addr_state),
		   unicast->is_infinite ? " infinite" : "");

		count++;
	}

	if (count == 0) {
		PR("\t<none>\n");
	}

	count = 0;

	PR("IPv4 multicast addresses (max %d):\n", NET_IF_MAX_IPV4_MADDR);
	ARRAY_FOR_EACH(ipv4->mcast, i) {
		mcast = &ipv4->mcast[i];

		if (!mcast->is_used) {
			continue;
		}

		PR("\t%s%s\n", net_sprint_ipv4_addr(&mcast->address.in_addr),
		   net_if_ipv4_maddr_is_joined(mcast) ? "" : "  <not joined>");

		count++;
	}

	if (count == 0) {
		PR("\t<none>\n");
	}

skip_ipv4:

	if (ipv4) {
		PR("IPv4 gateway : %s\n",
		   net_sprint_ipv4_addr(&ipv4->gw));
	}
#endif /* CONFIG_NET_IPV4 */

#if defined(CONFIG_NET_DHCPV4)
	if (net_if_flag_is_set(iface, NET_IF_IPV4)) {
		if (iface->config.dhcpv4.state != NET_DHCPV4_DISABLED) {
			PR("DHCPv4 lease time : %u\n",
			   iface->config.dhcpv4.lease_time);
			PR("DHCPv4 renew time : %u\n",
			   iface->config.dhcpv4.renewal_time);
			PR("DHCPv4 server     : %s\n",
			   net_sprint_ipv4_addr(&iface->config.dhcpv4.server_id));
			PR("DHCPv4 requested  : %s\n",
			   net_sprint_ipv4_addr(&iface->config.dhcpv4.requested_ip));
			PR("DHCPv4 state      : %s\n",
			   net_dhcpv4_state_name(iface->config.dhcpv4.state));
			PR("DHCPv4 attempts   : %d\n",
			   iface->config.dhcpv4.attempts);
		}

		PR("DHCPv4 state      : %s\n",
		   net_dhcpv4_state_name(iface->config.dhcpv4.state));
	}
#endif /* CONFIG_NET_DHCPV4 */
}

static int cmd_net_set_mac(const struct shell *sh, size_t argc, char *argv[])
{
#if !defined(CONFIG_NET_L2_ETHERNET) || !defined(CONFIG_NET_L2_ETHERNET_MGMT)
	PR_WARNING("Unsupported command, please enable CONFIG_NET_L2_ETHERNET "
		"and CONFIG_NET_L2_ETHERNET_MGMT\n");
	return -ENOEXEC;
#else
	struct net_if *iface;
	struct ethernet_req_params params;
	char *mac_addr = params.mac_address.addr;
	int idx;
	int ret;

	if (argc < 3) {
		PR_WARNING("Missing interface index and/or MAC address\n");
		goto err;
	}

	idx = get_iface_idx(sh, argv[1]);
	if (idx < 0) {
		goto err;
	}

	iface = net_if_get_by_index(idx);
	if (!iface) {
		PR_WARNING("No such interface in index %d\n", idx);
		goto err;
	}

	if (net_if_l2(iface) != &NET_L2_GET_NAME(ETHERNET)) {
		PR_WARNING("MAC address can be set only for Ethernet\n");
		goto err;
	}

	if (!strncasecmp(argv[2], "random", 6)) {
		sys_rand_get(mac_addr, NET_ETH_ADDR_LEN);
		mac_addr[0] = (mac_addr[0] & UNICAST_MASK) | LOCAL_BIT;
	} else {
		if ((net_bytes_from_str(mac_addr, sizeof(params.mac_address), argv[2]) < 0) ||
		    !net_eth_is_addr_valid(&params.mac_address)) {
			PR_WARNING("Invalid MAC address: %s\n", argv[2]);
			goto err;
		}
	}

	ret = net_mgmt(NET_REQUEST_ETHERNET_SET_MAC_ADDRESS, iface, &params, sizeof(params));
	if (ret < 0) {
		if (ret == -EACCES) {
			PR_WARNING("MAC address cannot be set when interface is operational\n");
			goto err;
		}
		PR_WARNING("Failed to set MAC address (%d)\n", ret);
		goto err;
	}

	PR_INFO("MAC address set to %s\n",
		net_sprint_ll_addr(net_if_get_link_addr(iface)->addr,
		net_if_get_link_addr(iface)->len));

	return 0;
err:
	return -ENOEXEC;
#endif /* CONFIG_NET_L2_ETHERNET */
}

static int cmd_net_iface_up(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface;
	int idx, ret;

	idx = get_iface_idx(sh, argv[1]);
	if (idx < 0) {
		return -ENOEXEC;
	}

	iface = net_if_get_by_index(idx);
	if (!iface) {
		PR_WARNING("No such interface in index %d\n", idx);
		return -ENOEXEC;
	}

	if (net_if_is_up(iface)) {
		PR_WARNING("Interface %d is already up.\n", idx);
		return -ENOEXEC;
	}

	ret = net_if_up(iface);
	if (ret) {
		PR_WARNING("Cannot take interface %d up (%d)\n", idx, ret);
		return -ENOEXEC;
	}

	PR("Interface %d is up\n", idx);

	return 0;
}

static int cmd_net_iface_down(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface;
	int idx, ret;

	idx = get_iface_idx(sh, argv[1]);
	if (idx < 0) {
		return -ENOEXEC;
	}

	iface = net_if_get_by_index(idx);
	if (!iface) {
		PR_WARNING("No such interface in index %d\n", idx);
		return -ENOEXEC;
	}

	ret = net_if_down(iface);
	if (ret) {
		PR_WARNING("Cannot take interface %d down (%d)\n", idx, ret);
		return -ENOEXEC;
	}

	PR("Interface %d is down\n", idx);

	return 0;
}

static int cmd_net_iface(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface = NULL;
	struct net_shell_user_data user_data;
	int idx;

	if (argv[1]) {
		idx = get_iface_idx(sh, argv[1]);
		if (idx < 0) {
			return -ENOEXEC;
		}

		iface = net_if_get_by_index(idx);
		if (!iface) {
			PR_WARNING("No such interface in index %d\n", idx);
			return -ENOEXEC;
		}
	}

#if defined(CONFIG_NET_HOSTNAME_ENABLE)
	PR("Hostname: %s\n", net_hostname_get());
#endif
	PR("Default interface: %d\n\n",
	   net_if_get_by_iface(net_if_get_default()));

	user_data.sh = sh;
	user_data.user_data = iface;

	net_if_foreach(iface_cb, &user_data);

	return 0;
}

static int cmd_net_default_iface(const struct shell *sh, size_t argc, char *argv[])
{
	struct net_if *iface = NULL;
	int idx;

	if (argc < 2) {
		iface = net_if_get_default();
		if (!iface) {
			PR_WARNING("No default interface\n");
			return -ENOEXEC;
		}

		idx = net_if_get_by_iface(iface);
		PR("Default interface: %d\n", idx);
	} else {
		int new_idx;

		idx = get_iface_idx(sh, argv[1]);
		if (idx < 0) {
			return -ENOEXEC;
		}

		net_if_set_default(net_if_get_by_index(idx));

		new_idx = net_if_get_by_iface(net_if_get_default());
		if (new_idx != idx) {
			PR_WARNING("Failed to set default interface to %d\n", idx);
			return -ENOEXEC;
		}

		PR("Default interface: %d\n", new_idx);
	}

	return 0;
}

#if defined(CONFIG_ETH_PHY_DRIVER)
static int cmd_net_link_speed(const struct shell *sh, size_t argc, char *argv[])
{
	int idx = get_iface_idx(sh, argv[1]);
	const struct device *phy_dev;
	bool half_duplex = false;
	uint16_t user_input_spd;
	struct net_if *iface;
	uint16_t speed = 0U;
	uint16_t flags = 0U;
	int ret;

	if (argc < 3) {
		PR_WARNING("Usage: net iface set_link <index> "
			   "<Speed:10/100/1000/2500/5000> [Duplex]:h/f>\n");
		return -ENOEXEC;
	}

	iface = net_if_get_by_index(idx);
	if (net_if_l2(iface) != &NET_L2_GET_NAME(ETHERNET)) {
		PR_WARNING("Interface %d is not Ethernet type\n", idx);
		return -EINVAL;
	}

	phy_dev = net_eth_get_phy(iface);
	if (!phy_dev) {
		PR_WARNING("No PHY device found for interface %d\n", idx);
		return -ENOEXEC;
	}

	for (int k = 2; k < argc; k++) {
		if ((k + 1 < argc) && (argv[k+1][0] == 'h')) {
			half_duplex = true;
		} else {
			half_duplex = false;
		}

		user_input_spd = shell_strtoul(argv[k], 10, &ret);
		switch (user_input_spd) {
		case 0:
			if (strcmp(argv[k], "no-autoneg") == 0) {
				flags |= PHY_FLAG_AUTO_NEGOTIATION_DISABLED;
				continue;
			}
			break;
		case 10:
			speed |= half_duplex ? LINK_HALF_10BASE : LINK_FULL_10BASE;
			break;
		case 100:
			speed |= half_duplex ? LINK_HALF_100BASE : LINK_FULL_100BASE;
			break;
		case 1000:
			speed |= half_duplex ? LINK_HALF_1000BASE :  LINK_FULL_1000BASE;
			break;
		case 2500:
			if (half_duplex) {
				PR_WARNING("2500BASE half-duplex not supported\n");
				return -ENOTSUP;
			}
			speed |= LINK_FULL_2500BASE;
			break;
		case 5000:
			if (half_duplex) {
				PR_WARNING("5000BASE half-duplex not supported\n");
				return -ENOTSUP;
			}
			speed |= LINK_FULL_5000BASE;
			break;
		default:
			PR_WARNING("Unsupported speed %d\n", user_input_spd);
			return -ENOTSUP;
		}
	}

	if (speed != 0U) {
		return phy_configure_link(phy_dev, speed, flags);
	}
	PR_WARNING("No speed specified\n");
	return -ENOEXEC;
}
#endif /* CONFIG_ETH_PHY_DRIVER */

#if defined(CONFIG_NET_SHELL_DYN_CMD_COMPLETION)

#include "iface_dynamic.h"

#endif /* CONFIG_NET_SHELL_DYN_CMD_COMPLETION */

SHELL_STATIC_SUBCMD_SET_CREATE(net_cmd_iface,
	SHELL_CMD(up, IFACE_DYN_CMD,
		  "'net iface up <index>' takes network interface up.",
		  cmd_net_iface_up),
	SHELL_CMD(down, IFACE_DYN_CMD,
		  "'net iface down <index>' takes network interface "
		  "down.",
		  cmd_net_iface_down),
	SHELL_CMD(show, IFACE_DYN_CMD,
		  "'net iface <index>' shows network interface "
		  "information.",
		  cmd_net_iface),
	SHELL_CMD(set_mac, IFACE_DYN_CMD,
		  "'net iface set_mac <index> <MAC>' sets MAC address for the network interface.",
		  cmd_net_set_mac),
	SHELL_CMD(default, IFACE_DYN_CMD,
		  "'net iface default [<index>]' displays or sets the default network interface.",
		  cmd_net_default_iface),
#if defined(CONFIG_ETH_PHY_DRIVER)
	SHELL_CMD(set_link, IFACE_DYN_CMD,
		  "'net iface set_link <index> <Speed 10/100/1000/2500/5000> "
		  "<Duplex[optional]:h/f>'"
		  " sets link speed for the network interface.",
		  cmd_net_link_speed),
#endif /* CONFIG_ETH_PHY_DRIVER */
	SHELL_SUBCMD_SET_END
);

SHELL_SUBCMD_ADD((net), iface, &net_cmd_iface,
		 "Print information about network interfaces.",
		 cmd_net_iface, 1, 1);
