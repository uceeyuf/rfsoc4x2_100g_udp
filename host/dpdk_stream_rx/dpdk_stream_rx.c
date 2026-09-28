// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Yijie Yu
//
// dpdk_stream_rx - receive the DDR record stream of the FPGA with DPDK (Linux, mlx5 PMD) and
// check every sample.
//
// Stream packets (FPGA :1236+k -> PC :1237+k, flow k): UDP payload
//   u32 start_index | u32 total_samples | 14 bytes 0 | samples (u32, little endian)
// With the ramp test data (ddr_record_loop TEST_RAMP=1) sample j of a packet is start_index + j.
// Packet n goes out on flow n mod F and start_index runs over all flows, so within one flow
// start_index advances by F x samples-per-packet (mod total_samples) from packet to packet;
// a larger step counts the packets in between as lost. A flow always lands on the same RSS
// queue, so each receive core keeps the state of its own flows.
//
// The FPGA streams to whoever sent it the last UDP packet: the program sends one to
// FPGA:1237 at start. Start and stop the stream with scripts/vio_speed.tcl.
//
//   sudo ./dpdk_stream_rx -l 0-8 -a 0000:02:00.0 -- [--rxq 8] [--seconds 30]
//        [--fpga-ip 192.168.100.1] [--local-ip 192.168.100.2] [--port 1237]
//
// Prints one line per second (packets, UDP payload Gbit/s, lost, bad samples, NIC drops), the
// totals, and the mean rate over the full seconds (the first and last second with data left out).

#include <inttypes.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>

#include <rte_arp.h>
#include <rte_cycles.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_launch.h>
#include <rte_lcore.h>
#include <rte_mbuf.h>
#include <rte_udp.h>

#define MAX_FLOWS  32
#define MAX_RXQ    32
#define BURST      64
#define REC_HDR    22
#define MBUF_DATA  (9216 + RTE_PKTMBUF_HEADROOM)

static struct {
    uint16_t port_id, udp_port;
    uint32_t fpga_ip, local_ip;          // host byte order
    int rxq, rxd;
    double seconds;
} O = {.port_id = 0, .udp_port = 1237, .rxq = 8, .rxd = 4096, .seconds = 30};

struct flow {                            // per flow, touched only by the core of its queue
    bool seen, step_known;
    uint32_t last, step;
};

struct qstat {                           // per receive core, read by the main core
    _Atomic uint64_t pkts, bytes, lost, bad, other, restarts;
} __rte_cache_aligned;

static struct qstat qs[MAX_RXQ];
static struct rte_mempool *pool;
static struct rte_ether_addr my_mac;
static _Atomic bool quit;
static _Atomic uint32_t total_samples;

static void on_signal(int s) { (void)s; atomic_store(&quit, true); }

static int parse_ip(const char *s, uint32_t *ip)
{
    struct in_addr a;
    if (inet_pton(AF_INET, s, &a) != 1) return -1;
    *ip = ntohl(a.s_addr);
    return 0;
}

// ---------------------------------------------------------------- packets
static bool samples_ok(const uint8_t *p, uint32_t n, uint32_t first)
{
    uint32_t diff = 0;
    for (uint32_t j = 0; j < n; j++) {
        uint32_t v;
        memcpy(&v, p + 4 * j, 4);
        diff |= v ^ (first + j);
    }
    return diff == 0;
}

static void on_stream(struct qstat *st, struct flow *fl, const uint8_t *p, uint32_t len)
{
    uint32_t si, total;
    memcpy(&si, p, 4); memcpy(&total, p + 4, 4);
    uint32_t ns = (len - REC_HDR) / 4;
    atomic_fetch_add_explicit(&st->pkts, 1, memory_order_relaxed);
    atomic_fetch_add_explicit(&st->bytes, len, memory_order_relaxed);
    if (total == 0 || !samples_ok(p + REC_HDR, ns, si))
        atomic_fetch_add_explicit(&st->bad, 1, memory_order_relaxed);
    if (total) atomic_store_explicit(&total_samples, total, memory_order_relaxed);
    if (fl->seen && total) {
        uint32_t d = (si + total - fl->last) % total;
        if (!fl->step_known) {
            fl->step = d;                // first two packets of the flow: F x samples per packet
            fl->step_known = d != 0;
        } else if (d != fl->step && fl->step) {
            if (si < fl->step) {
                // the stream was stopped and started again: flow k starts over at k x samples per packet
                atomic_fetch_add_explicit(&st->restarts, 1, memory_order_relaxed);
            } else {
                // packets missing in between
                uint32_t k = (d + total - fl->step) % total / fl->step;
                atomic_fetch_add_explicit(&st->lost, k, memory_order_relaxed);
            }
        }
    }
    fl->seen = true;
    fl->last = si;
}

// UDP payload of a stream packet, or NULL; *flow = its flow
static const uint8_t *stream_payload(struct rte_mbuf *m, uint32_t *len, int *flow)
{
    struct rte_ether_hdr *e = rte_pktmbuf_mtod(m, struct rte_ether_hdr *);
    if (e->ether_type != rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4) || m->data_len < 42 + REC_HDR) return NULL;
    struct rte_ipv4_hdr *ip = (struct rte_ipv4_hdr *)(e + 1);
    if (ip->next_proto_id != IPPROTO_UDP) return NULL;
    struct rte_udp_hdr *u = (struct rte_udp_hdr *)((uint8_t *)ip + (ip->version_ihl & 0xf) * 4);
    uint16_t dport = rte_be_to_cpu_16(u->dst_port);
    if (dport < O.udp_port || dport >= O.udp_port + MAX_FLOWS) return NULL;
    uint32_t ulen = rte_be_to_cpu_16(u->dgram_len);
    if (ulen < 8 + REC_HDR || (uint8_t *)u + ulen > rte_pktmbuf_mtod(m, uint8_t *) + m->data_len) return NULL;
    *len = ulen - 8;
    *flow = dport - O.udp_port;
    return (const uint8_t *)(u + 1);
}

static void send_frame(struct rte_mbuf *m)
{
    while (rte_eth_tx_burst(O.port_id, 0, &m, 1) == 0) {}
}

// answer ARP requests for our address (main core only)
static bool handle_arp(struct rte_mbuf *m)
{
    struct rte_ether_hdr *e = rte_pktmbuf_mtod(m, struct rte_ether_hdr *);
    if (e->ether_type != rte_cpu_to_be_16(RTE_ETHER_TYPE_ARP)) return false;
    struct rte_arp_hdr *a = (struct rte_arp_hdr *)(e + 1);
    if (rte_be_to_cpu_16(a->arp_opcode) != RTE_ARP_OP_REQUEST ||
        a->arp_data.arp_tip != rte_cpu_to_be_32(O.local_ip)) return true;
    struct rte_mbuf *r = rte_pktmbuf_alloc(pool);
    if (!r) return true;
    struct rte_ether_hdr *re = rte_pktmbuf_mtod(r, struct rte_ether_hdr *);
    struct rte_arp_hdr *ra = (struct rte_arp_hdr *)(re + 1);
    rte_ether_addr_copy(&a->arp_data.arp_sha, &re->dst_addr);
    rte_ether_addr_copy(&my_mac, &re->src_addr);
    re->ether_type = rte_cpu_to_be_16(RTE_ETHER_TYPE_ARP);
    *ra = *a;
    ra->arp_opcode = rte_cpu_to_be_16(RTE_ARP_OP_REPLY);
    rte_ether_addr_copy(&a->arp_data.arp_sha, &ra->arp_data.arp_tha);
    ra->arp_data.arp_tip = a->arp_data.arp_sip;
    rte_ether_addr_copy(&my_mac, &ra->arp_data.arp_sha);
    ra->arp_data.arp_sip = rte_cpu_to_be_32(O.local_ip);
    r->data_len = r->pkt_len = 60;
    memset((uint8_t *)(ra + 1), 0, 60 - 14 - sizeof(*ra));
    send_frame(r);
    return true;
}

// resolve the FPGA's MAC, then send it one UDP packet so that it streams to us
static void register_with_fpga(void)
{
    struct rte_ether_addr fpga_mac;
    bool known = false;
    for (int t = 0; t < 30 && !known && !atomic_load(&quit); t++) {
        struct rte_mbuf *m = rte_pktmbuf_alloc(pool);
        struct rte_ether_hdr *e = rte_pktmbuf_mtod(m, struct rte_ether_hdr *);
        struct rte_arp_hdr *a = (struct rte_arp_hdr *)(e + 1);
        memset(&e->dst_addr, 0xff, RTE_ETHER_ADDR_LEN);
        rte_ether_addr_copy(&my_mac, &e->src_addr);
        e->ether_type = rte_cpu_to_be_16(RTE_ETHER_TYPE_ARP);
        a->arp_hardware = rte_cpu_to_be_16(RTE_ARP_HRD_ETHER);
        a->arp_protocol = rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4);
        a->arp_hlen = RTE_ETHER_ADDR_LEN;
        a->arp_plen = 4;
        a->arp_opcode = rte_cpu_to_be_16(RTE_ARP_OP_REQUEST);
        rte_ether_addr_copy(&my_mac, &a->arp_data.arp_sha);
        a->arp_data.arp_sip = rte_cpu_to_be_32(O.local_ip);
        memset(&a->arp_data.arp_tha, 0, RTE_ETHER_ADDR_LEN);
        a->arp_data.arp_tip = rte_cpu_to_be_32(O.fpga_ip);
        m->data_len = m->pkt_len = 60;
        memset((uint8_t *)(a + 1), 0, 60 - 14 - sizeof(*a));
        send_frame(m);
        uint64_t end = rte_get_tsc_cycles() + rte_get_tsc_hz() / 10;
        while (rte_get_tsc_cycles() < end && !known) {
            struct rte_mbuf *b[BURST];
            for (int q = 0; q < O.rxq; q++) {
                uint16_t n = rte_eth_rx_burst(O.port_id, (uint16_t)q, b, BURST);
                for (uint16_t k = 0; k < n; k++) {
                    struct rte_ether_hdr *re = rte_pktmbuf_mtod(b[k], struct rte_ether_hdr *);
                    struct rte_arp_hdr *ra = (struct rte_arp_hdr *)(re + 1);
                    if (re->ether_type == rte_cpu_to_be_16(RTE_ETHER_TYPE_ARP) &&
                        rte_be_to_cpu_16(ra->arp_opcode) == RTE_ARP_OP_REPLY &&
                        ra->arp_data.arp_sip == rte_cpu_to_be_32(O.fpga_ip)) {
                        fpga_mac = ra->arp_data.arp_sha;
                        known = true;
                    } else {
                        handle_arp(b[k]);
                    }
                }
                if (n) rte_pktmbuf_free_bulk(b, n);
            }
        }
    }
    if (!known) rte_exit(1, "no ARP reply from the FPGA\n");
    // one UDP packet to FPGA:port (payload "REG")
    struct rte_mbuf *m = rte_pktmbuf_alloc(pool);
    uint8_t *p = rte_pktmbuf_mtod(m, uint8_t *);
    memset(p, 0, 60);
    struct rte_ether_hdr *e = (struct rte_ether_hdr *)p;
    struct rte_ipv4_hdr *ip = (struct rte_ipv4_hdr *)(p + 14);
    struct rte_udp_hdr *u = (struct rte_udp_hdr *)(p + 34);
    rte_ether_addr_copy(&fpga_mac, &e->dst_addr);
    rte_ether_addr_copy(&my_mac, &e->src_addr);
    e->ether_type = rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4);
    ip->version_ihl = 0x45;
    ip->total_length = rte_cpu_to_be_16(20 + 8 + 3);
    ip->time_to_live = 64;
    ip->next_proto_id = IPPROTO_UDP;
    ip->src_addr = rte_cpu_to_be_32(O.local_ip);
    ip->dst_addr = rte_cpu_to_be_32(O.fpga_ip);
    ip->hdr_checksum = rte_ipv4_cksum(ip);
    u->src_port = rte_cpu_to_be_16(O.udp_port);
    u->dst_port = rte_cpu_to_be_16(O.udp_port);
    u->dgram_len = rte_cpu_to_be_16(8 + 3);
    memcpy(p + 42, "REG", 3);
    m->data_len = m->pkt_len = 60;
    send_frame(m);
    printf("FPGA MAC %02x:%02x:%02x:%02x:%02x:%02x, registered as the stream destination\n",
           fpga_mac.addr_bytes[0], fpga_mac.addr_bytes[1], fpga_mac.addr_bytes[2],
           fpga_mac.addr_bytes[3], fpga_mac.addr_bytes[4], fpga_mac.addr_bytes[5]);
}

// ---------------------------------------------------------------- receive cores
static int rx_main(void *arg)
{
    int q = (int)(uintptr_t)arg;
    struct qstat *st = &qs[q];
    struct flow fl[MAX_FLOWS];
    memset(fl, 0, sizeof(fl));
    struct rte_mbuf *b[BURST];
    while (!atomic_load_explicit(&quit, memory_order_relaxed)) {
        uint16_t n = rte_eth_rx_burst(O.port_id, (uint16_t)q, b, BURST);
        for (uint16_t k = 0; k < n; k++) {
            uint32_t len;
            int f;
            const uint8_t *p = stream_payload(b[k], &len, &f);
            if (p) on_stream(st, &fl[f], p, len);
            else atomic_fetch_add_explicit(&st->other, 1, memory_order_relaxed);
        }
        if (n) rte_pktmbuf_free_bulk(b, n);
    }
    return 0;
}

// ---------------------------------------------------------------- setup
static void port_init(void)
{
    struct rte_eth_dev_info info;
    if (rte_eth_dev_info_get(O.port_id, &info)) rte_exit(1, "rte_eth_dev_info_get failed\n");
    struct rte_eth_conf conf;
    memset(&conf, 0, sizeof(conf));
    conf.rxmode.mq_mode = RTE_ETH_MQ_RX_RSS;
    conf.rx_adv_conf.rss_conf.rss_hf = (RTE_ETH_RSS_IP | RTE_ETH_RSS_UDP) & info.flow_type_rss_offloads;
    conf.rxmode.mtu = 9000;
    if (rte_eth_dev_configure(O.port_id, (uint16_t)O.rxq, 1, &conf)) rte_exit(1, "rte_eth_dev_configure failed\n");
    rte_eth_dev_set_mtu(O.port_id, 9000);
    int socket = rte_eth_dev_socket_id(O.port_id);
    uint16_t nrxd = (uint16_t)O.rxd, ntxd = 512;
    rte_eth_dev_adjust_nb_rx_tx_desc(O.port_id, &nrxd, &ntxd);
    for (int q = 0; q < O.rxq; q++)
        if (rte_eth_rx_queue_setup(O.port_id, (uint16_t)q, nrxd, (unsigned)socket, NULL, pool))
            rte_exit(1, "rx queue %d setup failed\n", q);
    if (rte_eth_tx_queue_setup(O.port_id, 0, ntxd, (unsigned)socket, NULL)) rte_exit(1, "tx queue setup failed\n");
    if (rte_eth_dev_start(O.port_id)) rte_exit(1, "rte_eth_dev_start failed\n");
    rte_eth_macaddr_get(O.port_id, &my_mac);
    for (int i = 0; i < 100; i++) {
        struct rte_eth_link link;
        if (rte_eth_link_get_nowait(O.port_id, &link) == 0 && link.link_status == RTE_ETH_LINK_UP) {
            printf("link up, %u Mbps\n", link.link_speed);
            return;
        }
        rte_delay_ms(100);
    }
    rte_exit(1, "link down\n");
}

static void usage(void)
{
    fprintf(stderr, "options: --rxq N --rxd N --seconds X --fpga-ip A --local-ip A --port N\n");
    exit(1);
}

static void parse_args(int argc, char **argv)
{
    parse_ip("192.168.100.1", &O.fpga_ip);
    parse_ip("192.168.100.2", &O.local_ip);
    for (int i = 1; i < argc; i += 2) {
        const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
        if (!v) usage();
        if (!strcmp(a, "--rxq")) O.rxq = atoi(v);
        else if (!strcmp(a, "--rxd")) O.rxd = atoi(v);
        else if (!strcmp(a, "--seconds")) O.seconds = atof(v);
        else if (!strcmp(a, "--fpga-ip")) { if (parse_ip(v, &O.fpga_ip)) usage(); }
        else if (!strcmp(a, "--local-ip")) { if (parse_ip(v, &O.local_ip)) usage(); }
        else if (!strcmp(a, "--port")) O.udp_port = (uint16_t)atoi(v);
        else usage();
    }
    if (O.rxq < 1 || O.rxq > MAX_RXQ) usage();
}

static void totals(uint64_t v[5])
{
    memset(v, 0, 5 * sizeof(uint64_t));
    for (int q = 0; q < O.rxq; q++) {
        v[0] += atomic_load(&qs[q].pkts);  v[1] += atomic_load(&qs[q].bytes);
        v[2] += atomic_load(&qs[q].lost);  v[3] += atomic_load(&qs[q].bad);
        v[4] += atomic_load(&qs[q].other);
    }
}

int main(int argc, char **argv)
{
    int ret = rte_eal_init(argc, argv);
    if (ret < 0) rte_exit(1, "EAL init failed\n");
    parse_args(argc - ret, argv + ret);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    if (rte_eth_dev_count_avail() < 1) rte_exit(1, "no DPDK port (-a <pci address>)\n");
    if ((int)rte_lcore_count() < 1 + O.rxq)
        rte_exit(1, "need %d lcores (1 main + %d RX), have %u\n", 1 + O.rxq, O.rxq, rte_lcore_count());
    pool = rte_pktmbuf_pool_create("mbufs", (unsigned)(O.rxq * O.rxd + 8192), 512, 0, MBUF_DATA, rte_socket_id());
    if (!pool) rte_exit(1, "mbuf pool failed: hugepages?\n");
    port_init();
    register_with_fpga();

    unsigned lc = rte_get_next_lcore(-1, 1, 0);
    for (int q = 0; q < O.rxq; q++) {
        rte_eal_remote_launch(rx_main, (void *)(uintptr_t)q, lc);
        lc = rte_get_next_lcore(lc, 1, 0);
    }

    printf("%6s %10s %9s %9s %8s %8s %10s\n", "t", "packets", "Mpps", "Gbit/s", "lost", "bad", "NIC drop");
    uint64_t hz = rte_get_tsc_hz(), t0 = rte_get_tsc_cycles(), prev[5], cur[5], sum[5] = {0};
    struct rte_eth_stats s1, sp;
    rte_eth_stats_get(O.port_id, &sp);
    totals(prev);
    int active = 0, first = 0, last = 0;
    static double gbps[4096];
    for (int t = 1; t <= (int)O.seconds && !atomic_load(&quit); t++) {
        while (rte_get_tsc_cycles() < t0 + (uint64_t)t * hz && !atomic_load(&quit)) rte_delay_us_block(1000);
        totals(cur);
        rte_eth_stats_get(O.port_id, &s1);
        uint64_t d[5];
        for (int i = 0; i < 5; i++) d[i] = cur[i] - prev[i];
        uint64_t drop = (s1.imissed - sp.imissed) + (s1.ierrors - sp.ierrors) + (s1.rx_nombuf - sp.rx_nombuf);
        printf("%6d %10" PRIu64 " %9.3f %9.2f %8" PRIu64 " %8" PRIu64 " %10" PRIu64 "\n", t, d[0], d[0] / 1e6,
               d[1] * 8 / 1e9, d[2], d[3], drop);
        fflush(stdout);
        if (t < 4096) gbps[t] = d[1] * 8 / 1e9;
        if (d[0]) { active++; if (!first) first = t; last = t; for (int i = 0; i < 4; i++) sum[i] += d[i]; sum[4] += drop; }
        memcpy(prev, cur, sizeof(prev));
        sp = s1;
    }
    atomic_store(&quit, true);
    rte_eal_mp_wait_lcore();
    double steady = 0;
    for (int t = first + 1; t < last && t < 4096; t++) steady += gbps[t];
    if (last - first > 1) steady /= last - first - 1;
    printf("total over %d s with data: %" PRIu64 " packets, %" PRIu64 " lost, %" PRIu64 " with wrong samples, %"
           PRIu64 " dropped by the NIC; %.2f Gbit/s UDP payload over the full seconds (record: %u samples)\n",
           active, sum[0], sum[2], sum[3], sum[4], steady, atomic_load(&total_samples));
    uint64_t rs = 0;
    for (int q = 0; q < O.rxq; q++) rs += atomic_load(&qs[q].restarts);
    if (rs) printf("stream restarts seen: %" PRIu64 " (flow starts after a stop, not counted as lost)\n", rs);
    // NIC counters of dropped / damaged frames, for the packets missing without an imissed
    int nx = rte_eth_xstats_get_names(O.port_id, NULL, 0);
    if (nx > 0) {
        struct rte_eth_xstat_name *nm = calloc((size_t)nx, sizeof(*nm));
        struct rte_eth_xstat *xv = calloc((size_t)nx, sizeof(*xv));
        rte_eth_xstats_get_names(O.port_id, nm, (unsigned)nx);
        rte_eth_xstats_get(O.port_id, xv, (unsigned)nx);
        printf("NIC counters (non-zero errors / discards since start):");
        for (int i = 0; i < nx; i++) {
            const char *n = nm[xv[i].id].name;
            if (xv[i].value && (strstr(n, "err") || strstr(n, "discard") || strstr(n, "drop") || strstr(n, "crc") ||
                                strstr(n, "miss") || strstr(n, "out_of_buffer") || strstr(n, "undersize") ||
                                strstr(n, "oversize") || strstr(n, "symbol") || strstr(n, "fragment")))
                printf(" %s=%" PRIu64, n, xv[i].value);
        }
        printf("\n");
        free(nm); free(xv);
    }
    rte_eth_dev_stop(O.port_id);
    rte_eth_dev_close(O.port_id);
    rte_eal_cleanup();
    return 0;
}
