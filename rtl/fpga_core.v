// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Yijie Yu
//
// RFSoC 4x2 100G core: 512-bit datapath end to end.
//
//   DDR4-2400 64-bit -> MIG 512b @ 300 MHz -> async FIFO -> 512b @ 250 MHz (128 Gbps)
//     -> record_eth_tx (complete UDP frames) --+
//        udp_stack (ARP, ping, UDP echo) ------+-> arb mux -> async FIFO -> CMAC 512b @ 322 MHz
//
// Stream control (VIO): tx_speed_en start/stop, tx_delay idle cycles between packets,
// tx_length[4:0] flows, tx_length[5] rotate source IP, tx_length[11] 8 KB jumbo frames,
// bench_ctrl[0] sink mode (DDR read data is discarded at the core clock instead of sent),
// bench_ctrl[1] no DDR writes.
// Per-second counters (VIO inputs): MIG read / write beats, beats taken at the core clock,
// cycles the core clock side waited for DDR data, bytes handed to the CMAC.

`resetall
`timescale 1ns / 1ps
`default_nettype none

module fpga_core (
    input  wire         clk,            // 250 MHz (CLK_HZ): UDP stack, stream, DDR read side
    input  wire         rst,
    input  wire         clk_125,        // 125 MHz: DDS / DDR write side, CMAC init clock
    input  wire         rst_125,

    // QSFP28
    output wire         qsfp0_tx1_p, qsfp0_tx1_n,
    input  wire         qsfp0_rx1_p, qsfp0_rx1_n,
    output wire         qsfp0_tx2_p, qsfp0_tx2_n,
    input  wire         qsfp0_rx2_p, qsfp0_rx2_n,
    output wire         qsfp0_tx3_p, qsfp0_tx3_n,
    input  wire         qsfp0_rx3_p, qsfp0_rx3_n,
    output wire         qsfp0_tx4_p, qsfp0_tx4_n,
    input  wire         qsfp0_rx4_p, qsfp0_rx4_n,
    input  wire         qsfp0_mgt_refclk_0_p,
    input  wire         qsfp0_mgt_refclk_0_n,
    output wire         qsfp0_modsell,
    output wire         qsfp0_resetl,
    input  wire         qsfp0_modprsl,
    input  wire         qsfp0_intl,
    output wire         qsfp0_lpmode,

    // PL DDR4
    input  wire         c0_sys_clk_p,
    input  wire         c0_sys_clk_n,
    input  wire         ddr_sys_rst,
    output wire [16:0]  c0_ddr4_adr,
    output wire [1:0]   c0_ddr4_ba,
    output wire [0:0]   c0_ddr4_cke,
    output wire [0:0]   c0_ddr4_cs_n,
    inout  wire [7:0]   c0_ddr4_dm_dbi_n,
    inout  wire [63:0]  c0_ddr4_dq,
    inout  wire [7:0]   c0_ddr4_dqs_c,
    inout  wire [7:0]   c0_ddr4_dqs_t,
    output wire [0:0]   c0_ddr4_odt,
    output wire [0:0]   c0_ddr4_bg,
    output wire         c0_ddr4_reset_n,
    output wire         c0_ddr4_act_n,
    output wire [0:0]   c0_ddr4_ck_c,
    output wire [0:0]   c0_ddr4_ck_t
);

localparam CLK_HZ = 250000000;       // clk

// ---------------------------------------------------------------- configuration
localparam DW = 512, KW = DW / 8;
localparam [47:0] LOCAL_MAC   = 48'h02_00_00_00_00_00;
localparam [31:0] LOCAL_IP    = {8'd192, 8'd168, 8'd100, 8'd1};     // same plan as rfsoc4x2_corundum: board .1, PC .2
localparam [31:0] GATEWAY_IP  = {8'd192, 8'd168, 8'd100, 8'd254};
localparam [31:0] ALIAS_BASE  = {8'd192, 8'd168, 8'd100, 8'd128};   // .128-.159: flow addresses
localparam        ALIAS_COUNT = 32;
localparam [31:0] SUBNET_MASK = {8'd255, 8'd255, 8'd255, 8'd0};
localparam [15:0] STREAM_SRC_PORT = 16'd1236;
localparam [15:0] STREAM_DST_PORT = 16'd1237;
localparam REC_SAMPLES = 1048576;       // samples per DDR record (2^20, 4 MB)

assign qsfp0_modsell = 1'b1;
assign qsfp0_resetl  = 1'b1;
assign qsfp0_lpmode  = 1'b0;

// ---------------------------------------------------------------- CMAC
wire          mac_clk;                   // gt_txusrclk2 (322.265625 MHz), also the RX side
wire          mac_tx_rst, mac_rx_rst;
wire [DW-1:0] mac_tx_tdata, mac_rx_tdata;
wire [KW-1:0] mac_tx_tkeep, mac_rx_tkeep;
wire          mac_tx_tvalid, mac_tx_tready, mac_tx_tlast, mac_tx_tuser;
wire          mac_rx_tvalid, mac_rx_tlast, mac_rx_tuser;

cmac_usplus_0 cmac_inst (
    .gt_rxp_in({qsfp0_rx4_p, qsfp0_rx3_p, qsfp0_rx2_p, qsfp0_rx1_p}),
    .gt_rxn_in({qsfp0_rx4_n, qsfp0_rx3_n, qsfp0_rx2_n, qsfp0_rx1_n}),
    .gt_txp_out({qsfp0_tx4_p, qsfp0_tx3_p, qsfp0_tx2_p, qsfp0_tx1_p}),
    .gt_txn_out({qsfp0_tx4_n, qsfp0_tx3_n, qsfp0_tx2_n, qsfp0_tx1_n}),
    .gt_ref_clk_p(qsfp0_mgt_refclk_0_p),
    .gt_ref_clk_n(qsfp0_mgt_refclk_0_n),
    .gt_txusrclk2(mac_clk),
    .gt_loopback_in(12'd0),
    .gtwiz_reset_tx_datapath(1'b0),
    .gtwiz_reset_rx_datapath(1'b0),
    .sys_reset(rst_125),
    .init_clk(clk_125),
    .ctl_tx_rsfec_enable(1'b1),
    .ctl_rx_rsfec_enable(1'b1),
    .ctl_rsfec_ieee_error_indication_mode(1'b0),
    .ctl_rx_rsfec_enable_correction(1'b1),
    .ctl_rx_rsfec_enable_indication(1'b1),
    .rx_clk(mac_clk),
    .core_rx_reset(1'b0),
    .ctl_rx_enable(1'b1),
    .ctl_rx_force_resync(1'b0),
    .ctl_rx_test_pattern(1'b0),
    .usr_rx_reset(mac_rx_rst),
    .rx_axis_tvalid(mac_rx_tvalid),
    .rx_axis_tdata(mac_rx_tdata),
    .rx_axis_tlast(mac_rx_tlast),
    .rx_axis_tkeep(mac_rx_tkeep),
    .rx_axis_tuser(mac_rx_tuser),
    .core_tx_reset(1'b0),
    .ctl_tx_enable(1'b1),
    .ctl_tx_send_idle(1'b0),
    .ctl_tx_send_rfi(1'b0),
    .ctl_tx_send_lfi(1'b0),
    .ctl_tx_test_pattern(1'b0),
    .usr_tx_reset(mac_tx_rst),
    .tx_axis_tvalid(mac_tx_tvalid),
    .tx_axis_tready(mac_tx_tready),
    .tx_axis_tdata(mac_tx_tdata),
    .tx_axis_tlast(mac_tx_tlast),
    .tx_axis_tkeep(mac_tx_tkeep),
    .tx_axis_tuser(mac_tx_tuser),
    .tx_preamblein(56'd0),
    .core_drp_reset(1'b0),
    .drp_clk(1'b0),
    .drp_addr(10'd0),
    .drp_di(16'd0),
    .drp_en(1'b0),
    .drp_we(1'b0)
);

// ---------------------------------------------------------------- MAC <-> stack FIFOs (512 <-> 512)
wire [DW-1:0] rx_tdata, tx_tdata, txf_tdata;
wire [KW-1:0] rx_tkeep, tx_tkeep, txf_tkeep;
wire          rxf_overflow, rxf_bad_frame;       // mac_clk: frame dropped (FIFO full / bad FCS)
wire          rx_tvalid, rx_tready, rx_tlast, rx_tuser;
wire          tx_tvalid, tx_tready, tx_tlast, tx_tuser;
wire          txf_tvalid, txf_tready, txf_tlast, txf_tuser;

// 256 KB (~28 jumbo frames): absorbs host send bursts; 32 KB overflowed from ~64 Gbit/s
axis_async_fifo_adapter #(
    .DEPTH(262144),
    .S_DATA_WIDTH(DW), .S_KEEP_ENABLE(1), .S_KEEP_WIDTH(KW),
    .M_DATA_WIDTH(DW), .M_KEEP_ENABLE(1), .M_KEEP_WIDTH(KW),
    .ID_ENABLE(0), .DEST_ENABLE(0), .USER_ENABLE(1), .USER_WIDTH(1),
    .FRAME_FIFO(1), .DROP_OVERSIZE_FRAME(1), .DROP_BAD_FRAME(1), .DROP_WHEN_FULL(1)
)
rx_fifo (
    .s_clk(mac_clk), .s_rst(mac_rx_rst),
    .s_axis_tdata(mac_rx_tdata), .s_axis_tkeep(mac_rx_tkeep), .s_axis_tvalid(mac_rx_tvalid),
    .s_axis_tready(), .s_axis_tlast(mac_rx_tlast), .s_axis_tid(8'd0), .s_axis_tdest(8'd0),
    .s_axis_tuser(mac_rx_tuser),
    .m_clk(clk), .m_rst(rst),
    .m_axis_tdata(rx_tdata), .m_axis_tkeep(rx_tkeep), .m_axis_tvalid(rx_tvalid),
    .m_axis_tready(rx_tready), .m_axis_tlast(rx_tlast), .m_axis_tid(), .m_axis_tdest(),
    .m_axis_tuser(rx_tuser),
    .s_status_overflow(rxf_overflow), .s_status_bad_frame(rxf_bad_frame), .s_status_good_frame(),
    .m_status_overflow(), .m_status_bad_frame(), .m_status_good_frame()
);

axis_async_fifo_adapter #(
    .DEPTH(32768),
    .S_DATA_WIDTH(DW), .S_KEEP_ENABLE(1), .S_KEEP_WIDTH(KW),
    .M_DATA_WIDTH(DW), .M_KEEP_ENABLE(1), .M_KEEP_WIDTH(KW),
    .ID_ENABLE(0), .DEST_ENABLE(0), .USER_ENABLE(1), .USER_WIDTH(1),
    .FRAME_FIFO(1), .DROP_OVERSIZE_FRAME(1), .DROP_BAD_FRAME(1), .DROP_WHEN_FULL(0)
)
tx_fifo (
    .s_clk(clk), .s_rst(rst),
    .s_axis_tdata(tx_tdata), .s_axis_tkeep(tx_tkeep), .s_axis_tvalid(tx_tvalid),
    .s_axis_tready(tx_tready), .s_axis_tlast(tx_tlast), .s_axis_tid(8'd0), .s_axis_tdest(8'd0),
    .s_axis_tuser(tx_tuser),
    .m_clk(mac_clk), .m_rst(mac_tx_rst),
    .m_axis_tdata(txf_tdata), .m_axis_tkeep(txf_tkeep), .m_axis_tvalid(txf_tvalid),
    .m_axis_tready(txf_tready), .m_axis_tlast(txf_tlast), .m_axis_tid(), .m_axis_tdest(),
    .m_axis_tuser(txf_tuser),
    .s_status_overflow(), .s_status_bad_frame(), .s_status_good_frame(),
    .m_status_overflow(), .m_status_bad_frame(), .m_status_good_frame()
);

eth_pad_min #(.DATA_WIDTH(DW)) tx_pad (
    .clk(mac_clk), .rst(mac_tx_rst),
    .s_axis_tdata(txf_tdata), .s_axis_tkeep(txf_tkeep), .s_axis_tvalid(txf_tvalid),
    .s_axis_tready(txf_tready), .s_axis_tlast(txf_tlast), .s_axis_tuser(txf_tuser),
    .m_axis_tdata(mac_tx_tdata), .m_axis_tkeep(mac_tx_tkeep), .m_axis_tvalid(mac_tx_tvalid),
    .m_axis_tready(mac_tx_tready), .m_axis_tlast(mac_tx_tlast), .m_axis_tuser(mac_tx_tuser)
);

// ---------------------------------------------------------------- UDP/IP stack
wire [DW-1:0] sk_tdata, rs_tdata;
wire [KW-1:0] sk_tkeep;
wire          sk_tvalid, sk_tready, sk_tlast, sk_tuser;
wire          rs_tvalid, rs_tready, rs_tlast, rs_tuser;
wire [47:0]   stream_dest_mac;
wire [31:0]   stream_dest_ip;
wire          echo_drop, echo_good;

udp_stack #(
    .DATA_WIDTH(DW),
    .ECHO_PORT_A(16'd1234),
    .ECHO_PORT_B(16'd1235),
    .ECHO_FIFO_DEPTH(262144),
    .ALIAS_BASE(ALIAS_BASE),
    .ALIAS_COUNT(ALIAS_COUNT)
)
stack_inst (
    .clk(clk), .rst(rst),
    .local_mac(LOCAL_MAC), .local_ip(LOCAL_IP), .gateway_ip(GATEWAY_IP), .subnet_mask(SUBNET_MASK),
    .rx_axis_tdata(rx_tdata), .rx_axis_tkeep(rx_tkeep), .rx_axis_tvalid(rx_tvalid),
    .rx_axis_tready(rx_tready), .rx_axis_tlast(rx_tlast), .rx_axis_tuser(rx_tuser),
    .tx_axis_tdata(sk_tdata), .tx_axis_tkeep(sk_tkeep), .tx_axis_tvalid(sk_tvalid),
    .tx_axis_tready(sk_tready), .tx_axis_tlast(sk_tlast), .tx_axis_tuser(sk_tuser),
    .stream_dest_mac(stream_dest_mac), .stream_dest_ip(stream_dest_ip),
    .echo_drop(echo_drop), .echo_good(echo_good)
);

// stack frames and stream frames to the MAC
axis_arb_mux #(
    .S_COUNT(2), .DATA_WIDTH(DW), .KEEP_ENABLE(1), .KEEP_WIDTH(KW),
    .ID_ENABLE(0), .DEST_ENABLE(0), .USER_ENABLE(1), .USER_WIDTH(1), .LAST_ENABLE(1),
    .ARB_TYPE_ROUND_ROBIN(1), .ARB_LSB_HIGH_PRIORITY(1)
)
tx_mux (
    .clk(clk), .rst(rst),
    .s_axis_tdata({rs_tdata, sk_tdata}), .s_axis_tkeep({{KW{1'b1}}, sk_tkeep}),
    .s_axis_tvalid({rs_tvalid, sk_tvalid}), .s_axis_tready({rs_tready, sk_tready}),
    .s_axis_tlast({rs_tlast, sk_tlast}), .s_axis_tid(16'd0), .s_axis_tdest(16'd0),
    .s_axis_tuser({rs_tuser, sk_tuser}),
    .m_axis_tdata(tx_tdata), .m_axis_tkeep(tx_tkeep), .m_axis_tvalid(tx_tvalid),
    .m_axis_tready(tx_tready), .m_axis_tlast(tx_tlast), .m_axis_tid(), .m_axis_tdest(),
    .m_axis_tuser(tx_tuser)
);

// ---------------------------------------------------------------- control and counters (VIO)
wire        tx_speed_en;
wire [15:0] tx_length;
wire [15:0] tx_delay;
wire [7:0]  bench_ctrl;
wire [3:0]  txstate;
wire [31:0] rd_beats_per_s, wr_beats_per_s, out_beats_per_s, starve_per_s;
wire [39:0] mac_tx_bytes_per_s;
// echo path, per second: CMAC RX frames / flagged bad, RX FIFO drops (full / bad),
// frames and back-pressure cycles into the stack, echo FIFO drops / packets, CMAC TX frames
wire [31:0] mac_rx_frames_per_s, mac_rx_bad_per_s, rxf_drop_per_s, rxf_bad_per_s;
wire [31:0] stack_rx_frames_per_s, stack_stall_per_s, echo_drop_per_s, echo_good_per_s;
wire [31:0] mac_tx_frames_per_s;

wire sink_mode = bench_ctrl[0];
wire no_write  = bench_ctrl[1];

vio_0 vio_inst (
    .clk(clk),
    .probe_in0(txstate),
    .probe_in1(rd_beats_per_s),
    .probe_in2(wr_beats_per_s),
    .probe_in3(out_beats_per_s),
    .probe_in4(starve_per_s),
    .probe_in5(mac_tx_bytes_per_s),
    .probe_in6(mac_rx_frames_per_s),
    .probe_in7(mac_rx_bad_per_s),
    .probe_in8(rxf_drop_per_s),
    .probe_in9(rxf_bad_per_s),
    .probe_in10(stack_rx_frames_per_s),
    .probe_in11(stack_stall_per_s),
    .probe_in12(echo_drop_per_s),
    .probe_in13(echo_good_per_s),
    .probe_in14(mac_tx_frames_per_s),
    .probe_out0(tx_speed_en),
    .probe_out1(tx_length),
    .probe_out2(tx_delay),
    .probe_out3(bench_ctrl)
);

// ---------------------------------------------------------------- DDR4 data source
wire [DW-1:0] rec_tdata;
wire          rec_tvalid, rec_tready, rec_tready_stream;

ddr_record_loop #(
    .TOTAL_SAMPLES(REC_SAMPLES),
    .TEST_RAMP(1)                        // 1: sample index ramp, 0: DDS
)
ddr_inst (
    .clk(clk), .rst(rst),
    .clk_wr(clk_125), .rst_wr(rst_125),
    .trig(tx_speed_en),
    .c0_sys_clk_p(c0_sys_clk_p), .c0_sys_clk_n(c0_sys_clk_n),
    .sys_rst(ddr_sys_rst), .init_calib_complete(),
    .c0_ddr4_adr(c0_ddr4_adr), .c0_ddr4_ba(c0_ddr4_ba), .c0_ddr4_cke(c0_ddr4_cke),
    .c0_ddr4_cs_n(c0_ddr4_cs_n), .c0_ddr4_dm_dbi_n(c0_ddr4_dm_dbi_n), .c0_ddr4_dq(c0_ddr4_dq),
    .c0_ddr4_dqs_c(c0_ddr4_dqs_c), .c0_ddr4_dqs_t(c0_ddr4_dqs_t), .c0_ddr4_odt(c0_ddr4_odt),
    .c0_ddr4_bg(c0_ddr4_bg), .c0_ddr4_reset_n(c0_ddr4_reset_n), .c0_ddr4_act_n(c0_ddr4_act_n),
    .c0_ddr4_ck_c(c0_ddr4_ck_c), .c0_ddr4_ck_t(c0_ddr4_ck_t),
    .no_write(no_write), .rd_beats_per_s(rd_beats_per_s), .wr_beats_per_s(wr_beats_per_s),
    .m_axis_tdata(rec_tdata), .m_axis_tvalid(rec_tvalid), .m_axis_tready(rec_tready)
);

// sink mode: take every DDR word at the core clock and throw it away.
// While stopped (and not finishing a frame) the reads still in flight are drained, so a new
// start begins at the start of a record and start_index 0 really is sample 0.
wire rec_drain = !tx_speed_en && txstate != 4'd2;
assign rec_tready = rec_drain | (sink_mode ? tx_speed_en : rec_tready_stream);

record_eth_tx #(
    .TOTAL_SAMPLES(REC_SAMPLES),
    .DATA_BEATS_STD(16),                 // 1024 data bytes + 22-byte header = 1046-byte UDP payload
    .DATA_BEATS_JUMBO(128),              // 8192 data bytes + 22-byte header = 8214-byte UDP payload
    .LOOP(1)
)
stream_inst (
    .clk(clk), .rst(rst),
    .local_mac(LOCAL_MAC), .local_ip(tx_length[5] ? ALIAS_BASE : LOCAL_IP), .local_port(STREAM_SRC_PORT),
    .dest_mac(stream_dest_mac), .dest_ip(stream_dest_ip), .dest_port(STREAM_DST_PORT),
    .trig(tx_speed_en && !sink_mode), .gap_cycles(tx_delay),
    .flow_count(tx_length[4:0]), .flow_vary_ip(tx_length[5]), .jumbo(tx_length[11]),
    .txstate(txstate),
    .s_axis_tdata(rec_tdata), .s_axis_tvalid(rec_tvalid), .s_axis_tready(rec_tready_stream),
    .m_axis_tdata(rs_tdata), .m_axis_tkeep(), .m_axis_tvalid(rs_tvalid), .m_axis_tready(rs_tready),
    .m_axis_tlast(rs_tlast), .m_axis_tuser(rs_tuser)
);

// ---------------------------------------------------------------- per-second counters
rate_counter #(.CLK_HZ(CLK_HZ)) out_rate (
    .clk(clk), .rst(rst), .inc(rec_tvalid & rec_tready), .rate(out_beats_per_s)
);
rate_counter #(.CLK_HZ(CLK_HZ)) starve_rate (
    .clk(clk), .rst(rst), .inc(tx_speed_en & !rec_tvalid & (sink_mode | rec_tready_stream)),
    .rate(starve_per_s)
);

// bytes handed to the CMAC (popcount of tkeep, one pipeline stage)
function [6:0] popcount64(input [63:0] v);
    integer i;
    begin
        popcount64 = 7'd0;
        for (i = 0; i < 64; i = i + 1) popcount64 = popcount64 + v[i];
    end
endfunction

reg        mtx_fire = 1'b0;
reg [63:0] mtx_keep = 64'd0;
reg [6:0]  mtx_inc = 7'd0;
always @(posedge mac_clk) begin
    mtx_fire <= mac_tx_tvalid & mac_tx_tready;
    mtx_keep <= mac_tx_tkeep;
    mtx_inc  <= mtx_fire ? popcount64(mtx_keep) : 7'd0;
end

rate_counter #(.CLK_HZ(322265625), .INC_WIDTH(7), .WIDTH(40)) mac_tx_rate (
    .clk(mac_clk), .rst(mac_tx_rst), .inc(mtx_inc), .rate(mac_tx_bytes_per_s)
);

// echo path counters
rate_counter #(.CLK_HZ(322265625)) mac_rx_frames_rate (
    .clk(mac_clk), .rst(mac_rx_rst), .inc(mac_rx_tvalid & mac_rx_tlast), .rate(mac_rx_frames_per_s)
);
rate_counter #(.CLK_HZ(322265625)) mac_rx_bad_rate (
    .clk(mac_clk), .rst(mac_rx_rst), .inc(mac_rx_tvalid & mac_rx_tlast & mac_rx_tuser), .rate(mac_rx_bad_per_s)
);
rate_counter #(.CLK_HZ(322265625)) rxf_drop_rate (
    .clk(mac_clk), .rst(mac_rx_rst), .inc(rxf_overflow), .rate(rxf_drop_per_s)
);
rate_counter #(.CLK_HZ(322265625)) rxf_bad_rate (
    .clk(mac_clk), .rst(mac_rx_rst), .inc(rxf_bad_frame), .rate(rxf_bad_per_s)
);
rate_counter #(.CLK_HZ(CLK_HZ)) stack_rx_frames_rate (
    .clk(clk), .rst(rst), .inc(rx_tvalid & rx_tready & rx_tlast), .rate(stack_rx_frames_per_s)
);
rate_counter #(.CLK_HZ(CLK_HZ)) stack_stall_rate (
    .clk(clk), .rst(rst), .inc(rx_tvalid & !rx_tready), .rate(stack_stall_per_s)
);
rate_counter #(.CLK_HZ(CLK_HZ)) echo_drop_rate (
    .clk(clk), .rst(rst), .inc(echo_drop), .rate(echo_drop_per_s)
);
rate_counter #(.CLK_HZ(CLK_HZ)) echo_good_rate (
    .clk(clk), .rst(rst), .inc(echo_good), .rate(echo_good_per_s)
);
rate_counter #(.CLK_HZ(322265625)) mac_tx_frames_rate (
    .clk(mac_clk), .rst(mac_tx_rst), .inc(mac_tx_tvalid & mac_tx_tready & mac_tx_tlast), .rate(mac_tx_frames_per_s)
);

endmodule

`resetall
