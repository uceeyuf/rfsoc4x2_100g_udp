// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Yijie Yu
//
// dpdk_loopback - video loopback through the FPGA UDP echo with DPDK (Linux, mlx5 PMD).
//
// Packets: 16-byte header + frame bytes
//   u32 magic 'RFLB' | u32 frame id | u16 packet index | u16 packet count | u32 byte offset
// Frames of W x H RGB24 (a cycle of C generated frames sent in turn) go out as UDP packets
// on F flows (flow i: source port SPORT+i to FPGA address IP+i, port 1234); the FPGA echoes
// every packet, the receive cores reassemble every frame and compare it byte by byte with
// what was sent. A rate passes when every frame is intact and the frames went out on time.
//
// Cores: T transmit cores (flow i on core i mod T, one TX queue each), R receive cores (one
// RSS queue each, hashing on IP addresses and UDP ports); the main core answers ARP and runs
// the sweep.
//
//   sudo ./dpdk_loopback -l 0-12 -a 0000:02:00.0 -- [options]
//     --ip 192.168.100.128   first FPGA address (flow i -> ip+i; the FPGA answers .128-.159)
//     --one-ip               every flow to --ip itself (for an echo with one address)
//     --local-ip 192.168.100.2 --port 1234 --sport 6000 --flows 16
//     --res 3840x2160 --payload 8956 --cycle 20 --spread 1.0 --max-gbps 0
//     --fps 120 --seconds 10 | --sweep 120,160,200,...
//     --txq 4 --rxq 8        (needs 1 + txq + rxq lcores)
//     --ref stored|gen       compare with the stored frames (default), or with a pattern computed from
//                            frame id + byte offset (no reference reads; the picture is then noise)
//     --catchup              a core that fell behind sends the missed packets back to back (default:
//                            it shifts its schedule instead, catching up at most CATCHUP_US)
//     --out dir              summary.txt, sent.ppm / received.ppm of the highest passing rate

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <arpa/inet.h>

#include <rte_arp.h>
#include <rte_common.h>
#include <rte_cycles.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_launch.h>
#include <rte_lcore.h>
#include <rte_mbuf.h>
#include <rte_pause.h>
#include <rte_spinlock.h>
#include <rte_udp.h>
#include <rte_version.h>

// ---------------------------------------------------------------- DPDK 20.11 compatibility
#if RTE_VERSION < RTE_VERSION_NUM(21, 11, 0, 0)
#define RTE_ETH_MQ_RX_RSS      ETH_MQ_RX_RSS
#define RTE_ETH_MQ_TX_NONE     ETH_MQ_TX_NONE
#define RTE_ETH_RSS_IP         ETH_RSS_IP
#define RTE_ETH_RSS_UDP        ETH_RSS_UDP
#define RTE_ETH_LINK_UP        ETH_LINK_UP
#define ETH_DST(e)             ((e)->d_addr)
#define ETH_SRC(e)             ((e)->s_addr)
#else
#define ETH_DST(e)             ((e)->dst_addr)
#define ETH_SRC(e)             ((e)->src_addr)
#endif

#define MAGIC      0x424C4652u      // "RFLB"
#define HDR        16
#define PROBE_ID   0xFFFFFFFFu
#define L2L3L4     (14 + 20 + 8)
#define TX_DESC    4096
#define BURST      32
#define OPEN_AHEAD 32          // frames opened ahead of TX core 0 (the others may run ahead of it)
#define CATCHUP_US 10          // a TX core behind schedule sends at most this much of its backlog at once
#define MBUF_DATA  (9216 + RTE_PKTMBUF_HEADROOM)
#define MAX_FLOWS  64
#define MAX_STEPS  64

// ---------------------------------------------------------------- options
static struct {
    uint16_t port_id;
    uint32_t ip, local_ip;           // host byte order
    uint16_t udp_port, sport;
    int flows, width, height, payload, cycle, txq, rxq, rxd;
    double spread, max_gbps, seconds, drain_idle_ms, drain_max_s;
    int fps[MAX_STEPS], nfps;
    bool sweep, light;               // light: RX cores only track which packets arrived (no copy, no compare)
    bool gen, catchup;               // gen: payload is a pattern of frame id + offset; catchup: see header
    bool one_ip;                     // every flow to the same FPGA address (flows differ in source port)
    char out[512];
} O = {
    .port_id = 0, .udp_port = 1234, .sport = 6000, .flows = 16, .width = 3840, .height = 2160,
    .payload = 8956, .cycle = 20, .txq = 4, .rxq = 8, .rxd = 4096, .spread = 1.0, .max_gbps = 0,
    .seconds = 10, .drain_idle_ms = 100, .drain_max_s = 2.0,
    .fps = {120}, .nfps = 1, .out = "dpdk_out",
};

// ---------------------------------------------------------------- state
// A frame being received: which packets arrived, and whether any differed from what was sent.
// Each packet is compared with the sent frame when it arrives, so no frame data is kept.
struct slot {
    _Atomic int64_t tag;             // frame id held
    _Atomic int state;               // 0 free/done, 1 receiving, 2 completing
    _Atomic int count;
    _Atomic int bad;                 // a packet differed from the sent frame
    _Atomic uint64_t *bits;
    uint64_t t_sent;                 // TSC
};

static int frame_bytes, npkts, words, nslot;
static uint8_t **orig;               // cycle frames, RGB24
static struct slot *slots;
static uint8_t *disp;                // the capture frame as received (for received.ppm)
static _Atomic int64_t disp_id = -1; // capture frame id once it arrived intact
static _Atomic int64_t cap_id = -1;  // frame of this step whose packets are also copied to disp

static struct rte_mempool *pool;
static struct rte_ether_addr my_mac, fpga_mac;
static uint8_t tmpl_full[MAX_FLOWS][L2L3L4], tmpl_last[MAX_FLOWS][L2L3L4];
static rte_spinlock_t ctl_lock = RTE_SPINLOCK_INITIALIZER;   // control TX queue (ARP, probes)
static uint16_t ctl_txq;

static _Atomic bool stop_rx, quit;
static _Atomic int64_t st_frames_sent, st_ok, st_bad, st_pk_sent, st_pk_recv, st_by_sent, st_by_recv;
static _Atomic int64_t st_lat_sum_us, st_lat_max_us, st_pk_late, st_pk_stale, st_slip_us;
static _Atomic uint64_t last_rx_tsc;
static _Atomic uint64_t t_tx_end_rx = UINT64_MAX;       // TX end as seen by the RX cores (late packets)
static uint64_t t_start, t_tx_end, hz;
static _Atomic int tx_running;

// per RX queue: packets of ours, and which flows landed there (RSS spread), summed at the end of a step
static int64_t q_pk[64], q_flow[64][MAX_FLOWS];

struct run { int fps; double seconds; };
static struct run R;

struct result {
    int fps;
    int64_t frames_sent, ok, bad, pk_sent, pk_recv, by_sent, by_recv, pk_late, pk_stale;
    double tx_s, lat_avg, lat_max, drain_ms, slip_ms;
    uint64_t imissed, ierrors, nombuf;
    char xs[512];                    // non-zero drop-type xstats deltas
};

static void on_signal(int s) { (void)s; atomic_store(&quit, true); }

// ---------------------------------------------------------------- test frames
// --ref gen: 32-bit word i of frame id is seed(id) + i * K, so a payload is written and checked
// from its frame id and byte offset alone (any flipped bit, misplaced packet or wrong frame differs)
#define GEN_K 0x9E3779B1u
static inline uint32_t gen_seed(uint32_t id) { return id * 0x85EBCA77u ^ 0x5BD1E995u; }

static void gen_fill(uint8_t *p, uint32_t id, uint32_t off, uint32_t len)
{
    uint32_t w = gen_seed(id) + (off / 4) * GEN_K;
    for (uint32_t i = 0; i < len / 4; i++, w += GEN_K) memcpy(p + 4 * i, &w, 4);
}

static bool gen_differs(const uint8_t *p, uint32_t id, uint32_t off, uint32_t len)
{
    uint32_t w0 = gen_seed(id) + (off / 4) * GEN_K, diff = 0;
    for (uint32_t i = 0; i < len / 4; i++) {
        uint32_t v;
        memcpy(&v, p + 4 * i, 4);
        diff |= v ^ (w0 + i * GEN_K);
    }
    return diff != 0;
}

static void make_frame(uint8_t *f, int c)
{
    int w = O.width, h = O.height, C = O.cycle;
    int bx = (int)((0.5 + 0.4 * cos(6.2831853 * c / C)) * (w - h / 3.5));
    int by = (int)((0.5 + 0.4 * sin(12.566371 * c / C)) * (h - h / 3.5));
    int r = h / 7;
    for (int y = 0; y < h; y++) {
        uint8_t *p = f + (size_t)y * w * 3;
        for (int x = 0; x < w; x++, p += 3) {
            int bar = ((x + c * w / C) / (w / 16)) & 1;
            int cr = (x * 160 / w + c * 11) & 0xff, cg = (y * 160 / h + c * 23) & 0xff, cb = ((x + y) / 16 + c * 37) & 0xff;
            if (bar) { cr = cr * 3 / 4 + 48; cg = cg * 3 / 4 + 32; }
            int dx = x - bx - r, dy = y - by - r;
            if (dx * dx + dy * dy < r * r) { cr = 255; cg = 210; cb = 40; }
            p[0] = (uint8_t)cr; p[1] = (uint8_t)cg; p[2] = (uint8_t)cb;
        }
    }
    // frame number as a row of blocks (binary), top left
    for (int b = 0; b < 8; b++)
        for (int y = h / 20; y < h / 20 + h / 30; y++)
            for (int x = w / 20 + b * w / 40; x < w / 20 + b * w / 40 + w / 50; x++) {
                uint8_t v = ((c >> (7 - b)) & 1) ? 255 : 30;
                uint8_t *p = f + ((size_t)y * w + x) * 3;
                p[0] = p[1] = p[2] = v;
            }
}

static void save_ppm(const char *name, const uint8_t *f)
{
    char path[640];
    snprintf(path, sizeof(path), "%s/%s", O.out, name);
    FILE *fp = fopen(path, "wb");
    if (!fp) return;
    fprintf(fp, "P6\n%d %d\n255\n", O.width, O.height);
    fwrite(f, 1, (size_t)frame_bytes, fp);
    fclose(fp);
}

// ---------------------------------------------------------------- packet templates
static uint32_t flow_ip(int i) { return O.one_ip ? O.ip : O.ip + (uint32_t)i; }

static void build_template(uint8_t *t, int i, int udp_payload)
{
    struct rte_ether_hdr *e = (struct rte_ether_hdr *)t;
    struct rte_ipv4_hdr *ip = (struct rte_ipv4_hdr *)(t + 14);
    struct rte_udp_hdr *u = (struct rte_udp_hdr *)(t + 34);
    rte_ether_addr_copy(&fpga_mac, &ETH_DST(e));
    rte_ether_addr_copy(&my_mac, &ETH_SRC(e));
    e->ether_type = rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4);
    memset(ip, 0, sizeof(*ip));
    ip->version_ihl = 0x45;
    ip->total_length = rte_cpu_to_be_16((uint16_t)(20 + 8 + udp_payload));
    ip->fragment_offset = rte_cpu_to_be_16(0x4000);      // DF
    ip->time_to_live = 64;
    ip->next_proto_id = IPPROTO_UDP;
    ip->src_addr = rte_cpu_to_be_32(O.local_ip);
    ip->dst_addr = rte_cpu_to_be_32(flow_ip(i));
    ip->hdr_checksum = rte_ipv4_cksum(ip);
    u->src_port = rte_cpu_to_be_16((uint16_t)(O.sport + i));
    u->dst_port = rte_cpu_to_be_16(O.udp_port);
    u->dgram_len = rte_cpu_to_be_16((uint16_t)(8 + udp_payload));
    u->dgram_cksum = 0;
}

static void build_templates(void)
{
    int last = frame_bytes - (npkts - 1) * O.payload;
    for (int i = 0; i < O.flows; i++) {
        build_template(tmpl_full[i], i, HDR + O.payload);
        build_template(tmpl_last[i], i, HDR + last);
    }
}

// ---------------------------------------------------------------- control path (ARP, probes)
static void ctl_send(struct rte_mbuf *m)
{
    rte_spinlock_lock(&ctl_lock);
    while (rte_eth_tx_burst(O.port_id, ctl_txq, &m, 1) == 0)
        rte_pause();
    rte_spinlock_unlock(&ctl_lock);
}

static void send_arp(uint16_t op, const struct rte_ether_addr *dst_mac, uint32_t tip_be)
{
    struct rte_mbuf *m = rte_pktmbuf_alloc(pool);
    if (!m) return;
    struct rte_ether_hdr *e = rte_pktmbuf_mtod(m, struct rte_ether_hdr *);
    struct rte_arp_hdr *a = (struct rte_arp_hdr *)(e + 1);
    if (dst_mac) rte_ether_addr_copy(dst_mac, &ETH_DST(e));
    else memset(&ETH_DST(e), 0xff, RTE_ETHER_ADDR_LEN);
    rte_ether_addr_copy(&my_mac, &ETH_SRC(e));
    e->ether_type = rte_cpu_to_be_16(RTE_ETHER_TYPE_ARP);
    a->arp_hardware = rte_cpu_to_be_16(RTE_ARP_HRD_ETHER);
    a->arp_protocol = rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4);
    a->arp_hlen = RTE_ETHER_ADDR_LEN;
    a->arp_plen = 4;
    a->arp_opcode = rte_cpu_to_be_16(op);
    rte_ether_addr_copy(&my_mac, &a->arp_data.arp_sha);
    a->arp_data.arp_sip = rte_cpu_to_be_32(O.local_ip);
    if (dst_mac) rte_ether_addr_copy(dst_mac, &a->arp_data.arp_tha);
    else memset(&a->arp_data.arp_tha, 0, RTE_ETHER_ADDR_LEN);
    a->arp_data.arp_tip = tip_be;
    m->data_len = m->pkt_len = 60;                        // minimum frame without FCS
    memset((uint8_t *)(a + 1), 0, 60 - 14 - sizeof(*a));
    ctl_send(m);
}

static _Atomic bool fpga_mac_known;

// ARP: answer requests for our address, learn the FPGA MAC from replies. Returns true if consumed.
static bool handle_arp(struct rte_mbuf *m)
{
    struct rte_ether_hdr *e = rte_pktmbuf_mtod(m, struct rte_ether_hdr *);
    if (e->ether_type != rte_cpu_to_be_16(RTE_ETHER_TYPE_ARP)) return false;
    struct rte_arp_hdr *a = (struct rte_arp_hdr *)(e + 1);
    uint16_t op = rte_be_to_cpu_16(a->arp_opcode);
    if (op == RTE_ARP_OP_REQUEST && a->arp_data.arp_tip == rte_cpu_to_be_32(O.local_ip)) {
        struct rte_ether_addr sha = a->arp_data.arp_sha;
        send_arp(RTE_ARP_OP_REPLY, &sha, a->arp_data.arp_sip);
    } else if (op == RTE_ARP_OP_REPLY && rte_be_to_cpu_32(a->arp_data.arp_sip) == O.ip) {
        rte_ether_addr_copy(&a->arp_data.arp_sha, &fpga_mac);
        atomic_store(&fpga_mac_known, true);
    }
    return true;
}

// ---------------------------------------------------------------- receive
// every packet of the frame arrived: it is intact unless one of them differed from the sent frame
static void frame_complete(struct slot *s)
{
    int64_t lat = (int64_t)((rte_get_tsc_cycles() - s->t_sent) * 1000000.0 / hz);
    atomic_fetch_add(&st_lat_sum_us, lat);
    int64_t m = atomic_load(&st_lat_max_us);
    while (lat > m && !atomic_compare_exchange_weak(&st_lat_max_us, &m, lat)) {}
    bool ok = O.light || !atomic_load(&s->bad);
    atomic_fetch_add(ok ? &st_ok : &st_bad, 1);
    int64_t id = atomic_load(&s->tag);
    if (ok && !O.light && id == atomic_load(&cap_id)) atomic_store(&disp_id, id);
    atomic_store(&s->state, 0);
}

static void on_packet(const uint8_t *p, uint32_t n)
{
    if (n < HDR) return;
    uint32_t magic, id, off;
    uint16_t idx;
    memcpy(&magic, p, 4); memcpy(&id, p + 4, 4); memcpy(&idx, p + 8, 2); memcpy(&off, p + 12, 4);
    if (magic != MAGIC || id == PROBE_ID) return;
    uint32_t len = n - HDR;
    if (idx >= npkts || off != (uint32_t)idx * O.payload || off + len > (uint32_t)frame_bytes) return;
    atomic_fetch_add_explicit(&st_pk_recv, 1, memory_order_relaxed);
    atomic_fetch_add_explicit(&st_by_recv, n, memory_order_relaxed);
    struct slot *s = &slots[id % nslot];
    if (atomic_load(&s->tag) != (int64_t)id || atomic_load(&s->state) != 1) {
        atomic_fetch_add_explicit(&st_pk_stale, 1, memory_order_relaxed);    // its frame is no longer open
        return;
    }
    uint64_t bit = 1ull << (idx & 63);
    if (atomic_load_explicit(&s->bits[idx >> 6], memory_order_relaxed) & bit) return;
    if (!O.light) {
        // compare with the sent frame now (the cost of the copy it replaces); the flag is set
        // before this packet counts, so the packet completing the frame sees every mismatch
        if (O.gen ? gen_differs(p + HDR, id, off, len) : memcmp(p + HDR, orig[id % O.cycle] + off, len) != 0)
            atomic_store(&s->bad, 1);
        if ((int64_t)id == atomic_load_explicit(&cap_id, memory_order_relaxed)) memcpy(disp + off, p + HDR, len);
    }
    if (atomic_load(&s->tag) != (int64_t)id || atomic_load(&s->state) != 1) return;
    if (atomic_fetch_or(&s->bits[idx >> 6], bit) & bit) return;
    if (atomic_fetch_add(&s->count, 1) + 1 == npkts) {
        int one = 1;
        if (atomic_compare_exchange_strong(&s->state, &one, 2)) frame_complete(s);
    }
}

// UDP payload of an echoed packet (NULL if not one of ours); *flow = its flow index
static const uint8_t *udp_payload(struct rte_mbuf *m, uint32_t *len, int *flow)
{
    struct rte_ether_hdr *e = rte_pktmbuf_mtod(m, struct rte_ether_hdr *);
    if (e->ether_type != rte_cpu_to_be_16(RTE_ETHER_TYPE_IPV4) || m->data_len < L2L3L4) return NULL;
    struct rte_ipv4_hdr *ip = (struct rte_ipv4_hdr *)(e + 1);
    if (ip->next_proto_id != IPPROTO_UDP) return NULL;
    int ihl = (ip->version_ihl & 0xf) * 4;
    struct rte_udp_hdr *u = (struct rte_udp_hdr *)((uint8_t *)ip + ihl);
    uint16_t dport = rte_be_to_cpu_16(u->dst_port);
    if (dport < O.sport || dport >= O.sport + O.flows) return NULL;
    uint32_t ulen = rte_be_to_cpu_16(u->dgram_len);
    if (ulen < 8 || (uint8_t *)u + ulen > rte_pktmbuf_mtod(m, uint8_t *) + m->data_len) return NULL;
    *len = ulen - 8;
    if (flow) *flow = dport - O.sport;
    return (const uint8_t *)(u + 1);
}

static int rx_main(void *arg)
{
    uint16_t q = (uint16_t)(uintptr_t)arg;
    struct rte_mbuf *b[64];
    int64_t pk = 0, late = 0, flow[MAX_FLOWS] = {0};
    while (!atomic_load_explicit(&stop_rx, memory_order_relaxed)) {
        uint16_t n = rte_eth_rx_burst(O.port_id, q, b, 64);
        if (!n) { rte_pause(); continue; }
        uint64_t now = rte_get_tsc_cycles();
        atomic_store_explicit(&last_rx_tsc, now, memory_order_relaxed);
        bool is_late = now > atomic_load_explicit(&t_tx_end_rx, memory_order_relaxed);
        for (uint16_t k = 0; k < n; k++) {
            uint32_t len;
            int fi;
            const uint8_t *p;
            if (handle_arp(b[k])) continue;
            if ((p = udp_payload(b[k], &len, &fi))) {
                pk++; flow[fi]++; late += is_late;
                on_packet(p, len);
            }
        }
        rte_pktmbuf_free_bulk(b, n);
    }
    q_pk[q] = pk;
    memcpy(q_flow[q], flow, sizeof(flow));
    atomic_fetch_add(&st_pk_late, late);
    return 0;
}

// ---------------------------------------------------------------- send
static void open_slot(int64_t n, uint64_t t0)
{
    struct slot *s = &slots[n % nslot];
    atomic_store(&s->state, 0);
    for (int w = 0; w < words; w++) atomic_store_explicit(&s->bits[w], 0, memory_order_relaxed);
    atomic_store(&s->count, 0);
    atomic_store(&s->bad, 0);
    s->t_sent = t0;
    atomic_store(&s->tag, n);
    atomic_store(&s->state, 1);
}

static void tx_flush(uint16_t q, struct rte_mbuf **b, int *nb)
{
    int sent = 0;
    while (sent < *nb) {
        sent += rte_eth_tx_burst(O.port_id, q, b + sent, (uint16_t)(*nb - sent));
        if (sent < *nb) rte_pause();
    }
    *nb = 0;
}

static int tx_main(void *arg)
{
    int j = (int)(uintptr_t)arg, T = O.txq, F = O.flows;
    uint16_t q = (uint16_t)j;
    int64_t frames = (int64_t)(R.seconds * R.fps + 0.5);
    double frame_t = (double)hz / R.fps, pkt_t = O.spread * frame_t / npkts;
    double min_gap = O.max_gbps > 0 ? T * (HDR + O.payload + 8 + 20 + 14 + 4 + 20) * 8.0 * hz / (O.max_gbps * 1e9) : 0;
    double next_ok = 0, slip = 0, slack = (double)hz * CATCHUP_US / 1e6;
    struct rte_mbuf *b[BURST];
    int nb = 0;
    for (int64_t n = 0; n < frames && !atomic_load(&quit); n++) {
        int cyc = (int)(n % O.cycle);
        double t0 = (double)t_start + n * frame_t;
        if (j == 0 && n + OPEN_AHEAD < frames) open_slot(n + OPEN_AHEAD, (uint64_t)(t0 + OPEN_AHEAD * frame_t));
        int64_t pk = 0, by = 0;
        for (int k = 0; k < npkts; k++) {
            int fi = k % F;
            if (fi % T != j) continue;
            double due = t0 + slip + k * pkt_t, now = (double)rte_get_tsc_cycles();
            // behind by more than the slack: shift the schedule rather than send the backlog as a burst
            // the FPGA FIFOs would have to absorb (256 KB is ~21 us at 100 Gbit/s)
            if (!O.catchup && now - due > slack) { slip += now - due - slack; due = now - slack; }
            if (due < next_ok) due = next_ok;
            if (now < due) {
                if (nb) tx_flush(q, b, &nb);
                while ((double)rte_get_tsc_cycles() < due) rte_pause();
            }
            struct rte_mbuf *m;
            while (!(m = rte_pktmbuf_alloc(pool))) { if (nb) tx_flush(q, b, &nb); rte_pause(); }
            uint32_t off = (uint32_t)k * O.payload;
            uint32_t len = (uint32_t)(k + 1 < npkts ? O.payload : frame_bytes - (int)off);
            uint8_t *p = rte_pktmbuf_mtod(m, uint8_t *);
            memcpy(p, k + 1 < npkts ? tmpl_full[fi] : tmpl_last[fi], L2L3L4);
            uint8_t *h = p + L2L3L4;
            uint32_t magic = MAGIC, id = (uint32_t)n;
            uint16_t idx = (uint16_t)k, cnt = (uint16_t)npkts;
            memcpy(h, &magic, 4); memcpy(h + 4, &id, 4); memcpy(h + 8, &idx, 2); memcpy(h + 10, &cnt, 2);
            memcpy(h + 12, &off, 4);
            if (O.gen) gen_fill(h + HDR, id, off, len);
            else memcpy(h + HDR, orig[cyc] + off, len);
            m->data_len = m->pkt_len = L2L3L4 + HDR + len;
            b[nb++] = m;
            pk++; by += HDR + len;
            if (min_gap > 0) {
                // token bucket: at most max_gbps, at most 4 packets of burst per core
                double now = (double)rte_get_tsc_cycles(), floor = now - 4 * min_gap;
                next_ok = (next_ok > floor ? next_ok : floor) + min_gap;
                tx_flush(q, b, &nb);
            } else if (nb == BURST) {
                tx_flush(q, b, &nb);
            }
        }
        if (nb) tx_flush(q, b, &nb);
        atomic_fetch_add(&st_pk_sent, pk);
        atomic_fetch_add(&st_by_sent, by);
        if (j == 0) atomic_fetch_add(&st_frames_sent, 1);
    }
    int64_t su = (int64_t)(slip * 1e6 / hz), m = atomic_load(&st_slip_us);
    while (su > m && !atomic_compare_exchange_weak(&st_slip_us, &m, su)) {}
    atomic_fetch_sub(&tx_running, 1);
    return 0;
}

// ---------------------------------------------------------------- setup
static int parse_ip(const char *s, uint32_t *ip)
{
    struct in_addr a;
    if (inet_pton(AF_INET, s, &a) != 1) return -1;
    *ip = ntohl(a.s_addr);
    return 0;
}

static void usage(void)
{
    fprintf(stderr, "options: --ip A --local-ip A --port N --sport N --flows N --res WxH --payload N --cycle N\n"
                    "         --spread X --max-gbps X --fps N --seconds X --sweep a,b,c --txq N --rxq N --rxd N\n"
                    "         --drain-idle-ms X --drain-max X --light --ref stored|gen --catchup --one-ip --out DIR\n");
    exit(1);
}

static void parse_args(int argc, char **argv)
{
    parse_ip("192.168.100.128", &O.ip);
    parse_ip("192.168.100.2", &O.local_ip);
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
        if (!strcmp(a, "--light")) { O.light = true; continue; }
        if (!strcmp(a, "--catchup")) { O.catchup = true; continue; }
        if (!strcmp(a, "--one-ip")) { O.one_ip = true; continue; }
        if (!v) usage();
        if (!strcmp(a, "--ip")) { if (parse_ip(v, &O.ip)) usage(); }
        else if (!strcmp(a, "--local-ip")) { if (parse_ip(v, &O.local_ip)) usage(); }
        else if (!strcmp(a, "--port")) O.udp_port = (uint16_t)atoi(v);
        else if (!strcmp(a, "--sport")) O.sport = (uint16_t)atoi(v);
        else if (!strcmp(a, "--flows")) O.flows = atoi(v);
        else if (!strcmp(a, "--res")) { if (sscanf(v, "%dx%d", &O.width, &O.height) != 2) usage(); }
        else if (!strcmp(a, "--payload")) O.payload = atoi(v);
        else if (!strcmp(a, "--cycle")) O.cycle = atoi(v);
        else if (!strcmp(a, "--spread")) O.spread = atof(v);
        else if (!strcmp(a, "--max-gbps")) O.max_gbps = atof(v);
        else if (!strcmp(a, "--seconds")) O.seconds = atof(v);
        else if (!strcmp(a, "--txq")) O.txq = atoi(v);
        else if (!strcmp(a, "--rxq")) O.rxq = atoi(v);
        else if (!strcmp(a, "--rxd")) O.rxd = atoi(v);
        else if (!strcmp(a, "--drain-idle-ms")) O.drain_idle_ms = atof(v);
        else if (!strcmp(a, "--drain-max")) O.drain_max_s = atof(v);
        else if (!strcmp(a, "--ref")) { if (!strcmp(v, "gen")) O.gen = true; else if (strcmp(v, "stored")) usage(); }
        else if (!strcmp(a, "--out")) snprintf(O.out, sizeof(O.out), "%s", v);
        else if (!strcmp(a, "--fps")) { O.fps[0] = atoi(v); O.nfps = 1; O.sweep = false; }
        else if (!strcmp(a, "--sweep")) {
            O.nfps = 0; O.sweep = true;
            char buf[512]; snprintf(buf, sizeof(buf), "%s", v);
            for (char *t = strtok(buf, ","); t && O.nfps < MAX_STEPS; t = strtok(NULL, ",")) O.fps[O.nfps++] = atoi(t);
        } else usage();
        i++;
    }
    if (O.flows < 1 || O.flows > MAX_FLOWS || O.txq < 1 || O.rxq < 1 || O.rxq > 64 || O.rxd < 64 ||
        O.payload < 64 || O.payload > 8956 || O.cycle < 1 ||
        (O.gen && (O.payload % 4 || O.width * O.height * 3 % 4)))
        usage();
}

static void port_init(void)
{
    struct rte_eth_dev_info info;
    if (rte_eth_dev_info_get(O.port_id, &info)) rte_exit(1, "rte_eth_dev_info_get failed\n");
    struct rte_eth_conf conf;
    memset(&conf, 0, sizeof(conf));
    conf.rxmode.mq_mode = RTE_ETH_MQ_RX_RSS;
    conf.rx_adv_conf.rss_conf.rss_hf = (RTE_ETH_RSS_IP | RTE_ETH_RSS_UDP) & info.flow_type_rss_offloads;
    conf.txmode.mq_mode = RTE_ETH_MQ_TX_NONE;
#if RTE_VERSION >= RTE_VERSION_NUM(21, 11, 0, 0)
    conf.rxmode.mtu = 9000;
#else
    conf.rxmode.max_rx_pkt_len = 9018;
    conf.rxmode.offloads |= DEV_RX_OFFLOAD_JUMBO_FRAME;
#endif
    ctl_txq = (uint16_t)O.txq;
    if (rte_eth_dev_configure(O.port_id, (uint16_t)O.rxq, (uint16_t)(O.txq + 1), &conf))
        rte_exit(1, "rte_eth_dev_configure failed\n");
    rte_eth_dev_set_mtu(O.port_id, 9000);
    int socket = rte_eth_dev_socket_id(O.port_id);
    uint16_t nrxd = (uint16_t)(O.rxd > 65535 ? 65535 : O.rxd), ntxd = TX_DESC;
    if (rte_eth_dev_adjust_nb_rx_tx_desc(O.port_id, &nrxd, &ntxd) == 0 && nrxd != O.rxd) {
        printf("RX descriptors: %d requested, %u used (device limit)\n", O.rxd, nrxd);
        O.rxd = nrxd;
    }
    for (int q = 0; q < O.rxq; q++)
        if (rte_eth_rx_queue_setup(O.port_id, (uint16_t)q, (uint16_t)O.rxd, (unsigned)socket, NULL, pool))
            rte_exit(1, "rx queue %d setup failed\n", q);
    for (int q = 0; q <= O.txq; q++)
        if (rte_eth_tx_queue_setup(O.port_id, (uint16_t)q, TX_DESC, (unsigned)socket, NULL))
            rte_exit(1, "tx queue %d setup failed\n", q);
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

// poll every RX queue from the main core (before the RX cores run)
static void poll_all(bool (*on_udp)(const uint8_t *, uint32_t, int), int flow_hint)
{
    struct rte_mbuf *b[64];
    for (int q = 0; q < O.rxq; q++) {
        uint16_t n = rte_eth_rx_burst(O.port_id, (uint16_t)q, b, 64);
        for (uint16_t k = 0; k < n; k++) {
            uint32_t len;
            const uint8_t *p;
            if (handle_arp(b[k])) continue;
            if (on_udp && (p = udp_payload(b[k], &len, NULL))) on_udp(p, len, flow_hint);
        }
        if (n) rte_pktmbuf_free_bulk(b, n);
    }
}

static _Atomic bool probe_seen;
static bool probe_rx(const uint8_t *p, uint32_t n, int flow)
{
    (void)flow;
    uint32_t magic, id;
    if (n < HDR) return false;
    memcpy(&magic, p, 4); memcpy(&id, p + 4, 4);
    if (magic == MAGIC && id == PROBE_ID) atomic_store(&probe_seen, true);
    return true;
}

static void resolve_and_probe(void)
{
    for (int t = 0; t < 20 && !atomic_load(&fpga_mac_known); t++) {
        send_arp(RTE_ARP_OP_REQUEST, NULL, rte_cpu_to_be_32(O.ip));
        uint64_t end = rte_get_tsc_cycles() + hz / 10;
        while (rte_get_tsc_cycles() < end && !atomic_load(&fpga_mac_known)) poll_all(NULL, 0);
    }
    if (!atomic_load(&fpga_mac_known)) rte_exit(1, "no ARP reply from the FPGA\n");
    printf("FPGA MAC %02x:%02x:%02x:%02x:%02x:%02x\n", fpga_mac.addr_bytes[0], fpga_mac.addr_bytes[1],
           fpga_mac.addr_bytes[2], fpga_mac.addr_bytes[3], fpga_mac.addr_bytes[4], fpga_mac.addr_bytes[5]);
    build_templates();
    // one probe per flow: the FPGA resolves our MAC while answering the first one
    for (int i = 0; i < O.flows; i++) {
        bool ok = false;
        for (int t = 0; t < 10 && !ok; t++) {
            struct rte_mbuf *m = rte_pktmbuf_alloc(pool);
            uint8_t *p = rte_pktmbuf_mtod(m, uint8_t *);
            build_template(p, i, HDR);
            uint32_t magic = MAGIC, id = PROBE_ID;
            memset(p + L2L3L4, 0, HDR);
            memcpy(p + L2L3L4, &magic, 4); memcpy(p + L2L3L4 + 4, &id, 4);
            m->data_len = m->pkt_len = 60;                  // padded to the minimum frame
            memset(p + L2L3L4 + HDR, 0, 60 - L2L3L4 - HDR);
            atomic_store(&probe_seen, false);
            ctl_send(m);
            uint64_t end = rte_get_tsc_cycles() + hz / 5;
            while (rte_get_tsc_cycles() < end && !atomic_load(&probe_seen)) poll_all(probe_rx, i);
            ok = atomic_load(&probe_seen);
        }
        if (!ok) {
            struct in_addr a = {.s_addr = htonl(flow_ip(i))};
            rte_exit(1, "no echo from %s:%u (flow %d)\n", inet_ntoa(a), O.udp_port, i);
        }
    }
    printf("all %d flows echo\n", O.flows);
}

// ---------------------------------------------------------------- run / report
static int nxs;                                     // extended statistics of the port
static struct rte_eth_xstat_name *xs_names;

static void xstats_init(void)
{
    nxs = rte_eth_xstats_get_names(O.port_id, NULL, 0);
    if (nxs <= 0) { nxs = 0; return; }
    xs_names = calloc((size_t)nxs, sizeof(*xs_names));
    if (rte_eth_xstats_get_names(O.port_id, xs_names, (unsigned)nxs) != nxs) nxs = 0;
}

static void xstats_read(uint64_t *v)
{
    if (!nxs) return;
    struct rte_eth_xstat *x = calloc((size_t)nxs, sizeof(*x));
    int n = rte_eth_xstats_get(O.port_id, x, (unsigned)nxs);
    for (int i = 0; i < n && i < nxs; i++) if (x[i].id < (uint64_t)nxs) v[x[i].id] = x[i].value;
    free(x);
}

static bool drop_stat(const char *name)
{
    static const char *keys[] = {"miss", "discard", "out_of_buffer", "nombuf", "error", "drop"};
    for (unsigned k = 0; k < sizeof(keys) / sizeof(keys[0]); k++)
        if (strstr(name, keys[k])) return true;
    return false;
}

static struct result run_step(int fps)
{
    R.fps = fps; R.seconds = O.seconds;
    atomic_store(&st_frames_sent, 0); atomic_store(&st_ok, 0); atomic_store(&st_bad, 0);
    atomic_store(&st_pk_sent, 0); atomic_store(&st_pk_recv, 0); atomic_store(&st_by_sent, 0);
    atomic_store(&st_by_recv, 0); atomic_store(&st_lat_sum_us, 0); atomic_store(&st_lat_max_us, 0);
    atomic_store(&st_pk_late, 0); atomic_store(&st_pk_stale, 0); atomic_store(&st_slip_us, 0);
    memset(q_pk, 0, sizeof(q_pk)); memset(q_flow, 0, sizeof(q_flow));
    for (int s = 0; s < nslot; s++) { atomic_store(&slots[s].state, 0); atomic_store(&slots[s].tag, -1); }
    atomic_store(&stop_rx, false);
    atomic_store(&cap_id, (int64_t)(O.seconds * fps / 2));     // a frame from mid-run is kept for received.ppm
    atomic_store(&last_rx_tsc, 0);
    atomic_store(&t_tx_end_rx, UINT64_MAX);

    struct rte_eth_stats s0, s1;
    rte_eth_stats_get(O.port_id, &s0);
    uint64_t *x0 = calloc((size_t)nxs + 1, sizeof(uint64_t)), *x1 = calloc((size_t)nxs + 1, sizeof(uint64_t));
    xstats_read(x0);

    t_start = rte_get_tsc_cycles() + hz / 20;
    double frame_t = (double)hz / fps;
    for (int k = 0; k < OPEN_AHEAD; k++) open_slot(k, (uint64_t)(t_start + k * frame_t));

    unsigned lc = rte_get_next_lcore(-1, 1, 0);
    unsigned rx_lc[64], tx_lc[64];
    for (int q = 0; q < O.rxq; q++) { rx_lc[q] = lc; rte_eal_remote_launch(rx_main, (void *)(uintptr_t)q, lc); lc = rte_get_next_lcore(lc, 1, 0); }
    atomic_store(&tx_running, O.txq);
    for (int j = 0; j < O.txq; j++) { tx_lc[j] = lc; rte_eal_remote_launch(tx_main, (void *)(uintptr_t)j, lc); lc = rte_get_next_lcore(lc, 1, 0); }
    for (int j = 0; j < O.txq; j++) rte_eal_wait_lcore(tx_lc[j]);
    t_tx_end = rte_get_tsc_cycles();
    atomic_store(&t_tx_end_rx, t_tx_end);
    // drain: keep receiving until nothing has arrived for drain_idle_ms (at most drain_max_s)
    uint64_t idle = (uint64_t)(O.drain_idle_ms * hz / 1000), maxw = (uint64_t)(O.drain_max_s * hz);
    for (;;) {
        uint64_t now = rte_get_tsc_cycles(), last = atomic_load(&last_rx_tsc);
        uint64_t ref = last > t_tx_end ? last : t_tx_end;
        if (now - ref > idle || now - t_tx_end > maxw) break;
        rte_delay_us_block(200);
    }
    uint64_t last = atomic_load(&last_rx_tsc);
    atomic_store(&stop_rx, true);
    for (int q = 0; q < O.rxq; q++) rte_eal_wait_lcore(rx_lc[q]);

    rte_eth_stats_get(O.port_id, &s1);
    xstats_read(x1);

    struct result r = {.fps = fps};
    r.frames_sent = atomic_load(&st_frames_sent); r.ok = atomic_load(&st_ok); r.bad = atomic_load(&st_bad);
    r.pk_sent = atomic_load(&st_pk_sent); r.pk_recv = atomic_load(&st_pk_recv);
    r.by_sent = atomic_load(&st_by_sent); r.by_recv = atomic_load(&st_by_recv);
    r.pk_late = atomic_load(&st_pk_late); r.pk_stale = atomic_load(&st_pk_stale);
    r.tx_s = (double)(t_tx_end - t_start) / hz;
    r.slip_ms = atomic_load(&st_slip_us) / 1000.0;
    r.drain_ms = last > t_tx_end ? (double)(last - t_tx_end) * 1000.0 / hz : 0;
    int64_t done = r.ok + r.bad;
    r.lat_avg = done ? atomic_load(&st_lat_sum_us) / 1000.0 / done : 0;
    r.lat_max = atomic_load(&st_lat_max_us) / 1000.0;
    r.imissed = s1.imissed - s0.imissed;
    r.ierrors = s1.ierrors - s0.ierrors;
    r.nombuf = s1.rx_nombuf - s0.rx_nombuf;
    size_t o = 0;
    for (int i = 0; i < nxs && o < sizeof(r.xs) - 64; i++)
        if (x1[i] != x0[i] && drop_stat(xs_names[i].name))
            o += (size_t)snprintf(r.xs + o, sizeof(r.xs) - o, " %s=+%" PRIu64, xs_names[i].name, x1[i] - x0[i]);
    free(x0); free(x1);
    return r;
}

// second report line: where packets went on the host side
static void detail(char *buf, size_t n, const struct result *r)
{
    size_t o = (size_t)snprintf(buf, n, "    tx: schedule slip %.1f ms   rx: late %" PRId64 " (drain %.1f ms)  stale %" PRId64
                                "  imissed %" PRIu64 "  ierrors %" PRIu64 "  nombuf %" PRIu64 "%s%s\n    queues:",
                                r->slip_ms, r->pk_late, r->drain_ms, r->pk_stale, r->imissed, r->ierrors, r->nombuf,
                                r->xs[0] ? "  xstats:" : "", r->xs);
    for (int q = 0; q < O.rxq && o < n - 48; q++) {
        o += (size_t)snprintf(buf + o, n - o, " q%d %" PRId64 " [", q, q_pk[q]);
        bool first = true;
        for (int f = 0; f < O.flows && o < n - 16; f++)
            if (q_flow[q][f]) { o += (size_t)snprintf(buf + o, n - o, first ? "f%d" : " f%d", f); first = false; }
        o += (size_t)snprintf(buf + o, n - o, "]");
    }
}

static bool passed(const struct result *r)
{
    return r->frames_sent > 0 && r->ok == r->frames_sent && r->tx_s <= O.seconds * 1.02 + 1.0 / r->fps;
}

static void line(char *buf, size_t n, const struct result *r)
{
    double lost = r->pk_sent ? 100.0 * (r->pk_sent - r->pk_recv) / r->pk_sent : 0;
    snprintf(buf, n, "%dx%d @ %3d fps  frames %5" PRId64 " sent %5" PRId64 " intact %" PRId64 " corrupted %" PRId64
             " incomplete   packets lost %.4f %%   TX %.2f / RX %.2f Gbps   latency %.1f / %.1f ms",
             O.width, O.height, r->fps, r->frames_sent, r->ok, r->bad, r->frames_sent - r->ok - r->bad, lost,
             r->tx_s > 0 ? r->by_sent * 8 / r->tx_s / 1e9 : 0, r->tx_s > 0 ? r->by_recv * 8 / r->tx_s / 1e9 : 0,
             r->lat_avg, r->lat_max);
}

int main(int argc, char **argv)
{
    int ret = rte_eal_init(argc, argv);
    if (ret < 0) rte_exit(1, "EAL init failed\n");
    parse_args(argc - ret, argv + ret);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    hz = rte_get_tsc_hz();
    if (rte_eth_dev_count_avail() < 1) rte_exit(1, "no DPDK port (-a <pci address>)\n");
    if ((int)rte_lcore_count() < 1 + O.txq + O.rxq)
        rte_exit(1, "need %d lcores (1 main + %d TX + %d RX), have %u\n",
                 1 + O.txq + O.rxq, O.txq, O.rxq, rte_lcore_count());

    frame_bytes = O.width * O.height * 3;
    npkts = (frame_bytes + O.payload - 1) / O.payload;
    words = (npkts + 63) / 64;
    nslot = 256;                     // frames in flight; a slot holds only arrival bits, no frame data
    mkdir(O.out, 0755);

    unsigned nmbuf = (unsigned)(O.rxq * O.rxd + (O.txq + 1) * TX_DESC + 8192);
    pool = rte_pktmbuf_pool_create("mbufs", nmbuf, 512, 0, MBUF_DATA, rte_socket_id());
    if (!pool) rte_exit(1, "mbuf pool (%u x %d B) failed: hugepages?\n", nmbuf, MBUF_DATA);

    printf("generating %d frames of %dx%d ...\n", O.cycle, O.width, O.height);
    orig = calloc((size_t)O.cycle, sizeof(*orig));
    for (int c = 0; c < O.cycle; c++) {
        orig[c] = malloc((size_t)frame_bytes);
        make_frame(orig[c], c);
    }
    slots = calloc((size_t)nslot, sizeof(*slots));
    for (int s = 0; s < nslot; s++) {
        slots[s].bits = calloc((size_t)words, sizeof(uint64_t));
        atomic_store(&slots[s].tag, -1);
    }
    disp = malloc((size_t)frame_bytes);
    memset(disp, 0, (size_t)frame_bytes);                  // fault the pages in before the run

    port_init();
    xstats_init();
    resolve_and_probe();

    char path[640], buf[512];
    snprintf(path, sizeof(path), "%s/summary.txt", O.out);
    FILE *sum = fopen(path, "w");
    struct in_addr a = {.s_addr = htonl(O.ip)};
    snprintf(buf, sizeof(buf), "FPGA %s (+0..%d) port %u, %d flows   payload %d B   %d packets/frame   %g s per step   "
             "(DPDK, %d TX / %d RX cores, %d RX descriptors, %s%s)",
             inet_ntoa(a), O.flows - 1, O.udp_port, O.flows, O.payload, npkts, O.seconds, O.txq, O.rxq, O.rxd,
             O.light ? "light: arrival only, no copy / compare" : O.gen ? "inline compare, generated reference"
             : "inline compare, stored reference", O.catchup ? ", catch-up bursts" : "");
    puts(buf);
    if (sum) fprintf(sum, "%s\n", buf);

    struct result res[MAX_STEPS];
    int nres = 0, best = -1, fails = 0;
    int64_t best_disp = -1;
    for (int i = 0; i < O.nfps && !atomic_load(&quit); i++) {
        atomic_store(&disp_id, -1);
        res[nres] = run_step(O.fps[i]);
        line(buf, sizeof(buf), &res[nres]);
        puts(buf);
        char dbuf[4096];
        detail(dbuf, sizeof(dbuf), &res[nres]);
        puts(dbuf);
        if (sum) { fprintf(sum, "%s\n%s\n", buf, dbuf); fflush(sum); }
        if (passed(&res[nres])) {
            best = nres; fails = 0;
            // keep a received frame of the best rate so far
            int64_t d = atomic_load(&disp_id);
            if (d >= 0 && !O.gen) { save_ppm("received.ppm", disp); save_ppm("sent.ppm", orig[d % O.cycle]); best_disp = d; }
        } else if (O.sweep && ++fails >= 2) {
            nres++;
            break;
        }
        nres++;
        rte_delay_ms(500);
    }
    if (best >= 0) {
        snprintf(buf, sizeof(buf), "Ceiling: %dx%d @ %d fps, %.2f Gbps each way, all frames intact",
                 O.width, O.height, res[best].fps, res[best].by_recv * 8 / res[best].tx_s / 1e9);
        puts(buf);
        if (sum) fprintf(sum, "%s\n", buf);
        if (best_disp >= 0) printf("sent.ppm / received.ppm: frame %" PRId64 " of the %d fps run\n", best_disp, res[best].fps);
    }
    if (sum) fclose(sum);
    rte_eth_dev_stop(O.port_id);
    rte_eth_dev_close(O.port_id);
    rte_eal_cleanup();
    return 0;
}
