#include <zephyr/kernel.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include "ipv4.h"
#include "udp_internal.h"
#include "tcp.h"
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <string.h>

static void dump_hex(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if ((i % 16) == 0) {
            printk("%04x  ", (unsigned int)i);
        }
        printk("%02x ", buf[i]);
        if ((i % 16) == 15 || i == (len - 1)) {
            printk("\n");
        }
    }
}

static void dump_mac(const char *label, const uint8_t mac[6])
{
    printk("%s %02x:%02x:%02x:%02x:%02x:%02x\n",
           label, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void dump_ipv4_addr(const char *label, uint32_t be_addr)
{
#if 0
    uint32_t a = sys_be32_to_cpu(be_addr);
    printk("%s %u.%u.%u.%u\n", label,
           (a >> 24) & 0xff, (a >> 16) & 0xff, (a >> 8) & 0xff, a & 0xff);
#endif
}

void net_pkt_dump_layers(struct net_pkt *pkt)
{
    if (!pkt) {
        printk("pkt: (null)\n");
        return;
    }

    const size_t len = net_pkt_get_len(pkt);
    if (len == 0) {
        printk("pkt len=0\n");
        return;
    }

    /* For debug only: keep buffer reasonably sized */
    if (len > 2048) {
        printk("pkt too big (%u)\n", (unsigned int)len);
        return;
    }

    uint8_t buf[2048];

	struct net_pkt_cursor backup;
    /* Copy packet bytes out (handles multi-frag) */
    net_pkt_cursor_backup(pkt, &backup);
    net_pkt_cursor_init(pkt);

    int rc = net_pkt_read(pkt, buf, len);
    net_pkt_cursor_restore(pkt, &backup);

    if (rc < 0) {
        printk("net_pkt_read failed: %d\n", rc);
        return;
    }

    printk("=== net_pkt len=%u ===\n", (unsigned int)len);

    /* ---- Ethernet ---- */
    if (len < sizeof(struct net_eth_hdr)) {
        printk("Too short for Ethernet header\n");
        dump_hex(buf, len);
        return;
    }

    const struct net_eth_hdr *eth = (const struct net_eth_hdr *)buf;
    dump_mac("ETH dst:", eth->dst.addr);
    dump_mac("ETH src:", eth->src.addr);

    uint16_t etype = sys_be16_to_cpu(eth->type);
    printk("ETH type: 0x%04x\n", etype);

    size_t off = sizeof(struct net_eth_hdr);

    /* ---- IPv4 ---- */
    if (etype == NET_ETH_PTYPE_IP) {
        if (len < off + sizeof(struct net_ipv4_hdr)) {
            printk("Too short for IPv4 header\n");
            dump_hex(buf, len);
            return;
        }

        const struct net_ipv4_hdr *ip = (const struct net_ipv4_hdr *)(buf + off);
        uint8_t ihl = (ip->vhl & 0x0f) * 4U;
        uint8_t ver = (ip->vhl >> 4);

        if (ver != 4 || ihl < 20) {
            printk("Invalid IPv4 header (ver=%u ihl=%u)\n", ver, ihl);
            dump_hex(buf, len);
            return;
        }

        uint16_t tot_len = sys_be16_to_cpu(ip->len);
        printk("IPv4: ihl=%u ttl=%u proto=%u total_len=%u\n",
               ihl, ip->ttl, ip->proto, tot_len);
        dump_ipv4_addr("IPv4 src:", ip->src);
        dump_ipv4_addr("IPv4 dst:", ip->dst);

        off += ihl;

        /* ---- UDP ---- */
        if (ip->proto == IPPROTO_UDP) {
            if (len < off + sizeof(struct net_udp_hdr)) {
                printk("Too short for UDP header\n");
                dump_hex(buf, len);
                return;
            }

            const struct net_udp_hdr *udp = (const struct net_udp_hdr *)(buf + off);
            uint16_t sport = sys_be16_to_cpu(udp->src_port);
            uint16_t dport = sys_be16_to_cpu(udp->dst_port);
            uint16_t ulen  = sys_be16_to_cpu(udp->len);

            printk("UDP: %u -> %u len=%u\n", sport, dport, ulen);

            off += sizeof(struct net_udp_hdr);
            printk("Payload (%u bytes):\n", (unsigned int)(len - off));
            dump_hex(buf + off, len - off);
            return;
        }

        /* ---- TCP ---- */
        if (ip->proto == IPPROTO_TCP) {
            if (len < off + sizeof(struct net_tcp_hdr)) {
                printk("Too short for TCP header\n");
                dump_hex(buf, len);
                return;
            }

            const struct net_tcp_hdr *tcp = (const struct net_tcp_hdr *)(buf + off);
            uint16_t sport = sys_be16_to_cpu(tcp->src_port);
            uint16_t dport = sys_be16_to_cpu(tcp->dst_port);
//            uint32_t seq   = sys_be32_to_cpu(tcp->seq);
//            uint32_t ack   = sys_be32_to_cpu(tcp->ack);
            uint8_t  doff  = (tcp->offset >> 4) * 4U;
#if 0
            printk("TCP: %u -> %u seq=%u ack=%u hdr_len=%u flags=0x%02x\n",
                   sport, dport, (unsigned int)seq, (unsigned int)ack,
                   doff, tcp->flags);
#endif

            off += doff;
            printk("Payload (%u bytes):\n", (unsigned int)(len - off));
            dump_hex(buf + off, len - off);
            return;
        }

        printk("IPv4 proto=%u (not decoded)\n", ip->proto);
        dump_hex(buf, len);
        return;
    }

    /* ---- IPv6 (if you want) ---- */
    if (etype == NET_ETH_PTYPE_IPV6) {
        printk("IPv6 frame (decode not included here)\n");
        dump_hex(buf, len);
        return;
    }

    printk("Non-IP ethertype\n");
    dump_hex(buf, len);
}
