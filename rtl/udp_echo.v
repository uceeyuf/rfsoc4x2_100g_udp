// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Yijie Yu
//
// UDP echo: packets received on PORT_A or PORT_B are sent back to their source (ports
// swapped), from the address they were sent to: local_ip or one of the ALIAS_COUNT addresses
// from ALIAS_BASE (the ARP responder answers for those too). Sending several flows to
// different alias addresses spreads the replies over the receive queues of a PC that hashes
// on IP addresses only (RSS). All other received UDP packets are consumed and dropped.
//
// The receive side never waits for the transmit side: each packet is written into a
// frame FIFO as two header words followed by its payload, and whole packets are dropped
// when the FIFO is full. (Holding the receive path while the transmit side resolves the
// peer's MAC would block the ARP reply behind it.)
//   word 0: {source IP, source port, destination port}   word 1: {16'd0, destination IP, UDP length}

`resetall
`timescale 1ns / 1ps
`default_nettype none

module udp_echo #(
    parameter DATA_WIDTH = 64,
    parameter KEEP_WIDTH = DATA_WIDTH / 8,
    parameter [15:0] PORT_A = 16'd1234,
    parameter [15:0] PORT_B = 16'd1235,
    parameter [31:0] ALIAS_BASE = 32'd0,
    parameter ALIAS_COUNT = 0,
    parameter FIFO_DEPTH = 32768            // bytes, several maximum-size packets
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] local_ip,

    // received UDP (from the UDP/IP stack)
    input  wire        s_udp_hdr_valid,
    output wire        s_udp_hdr_ready,
    input  wire [31:0] s_udp_ip_source_ip,
    input  wire [31:0] s_udp_ip_dest_ip,
    input  wire [15:0] s_udp_source_port,
    input  wire [15:0] s_udp_dest_port,
    input  wire [15:0] s_udp_length,
    input  wire [DATA_WIDTH-1:0] s_udp_payload_axis_tdata,
    input  wire [KEEP_WIDTH-1:0] s_udp_payload_axis_tkeep,
    input  wire        s_udp_payload_axis_tvalid,
    output wire        s_udp_payload_axis_tready,
    input  wire        s_udp_payload_axis_tlast,
    input  wire        s_udp_payload_axis_tuser,

    // UDP to transmit
    output wire        m_udp_hdr_valid,
    input  wire        m_udp_hdr_ready,
    output wire [5:0]  m_udp_ip_dscp,
    output wire [1:0]  m_udp_ip_ecn,
    output wire [7:0]  m_udp_ip_ttl,
    output wire [31:0] m_udp_ip_source_ip,
    output wire [31:0] m_udp_ip_dest_ip,
    output wire [15:0] m_udp_source_port,
    output wire [15:0] m_udp_dest_port,
    output wire [15:0] m_udp_length,
    output wire [15:0] m_udp_checksum,
    output wire [DATA_WIDTH-1:0] m_udp_payload_axis_tdata,
    output wire [KEEP_WIDTH-1:0] m_udp_payload_axis_tkeep,
    output wire        m_udp_payload_axis_tvalid,
    input  wire        m_udp_payload_axis_tready,
    output wire        m_udp_payload_axis_tlast,
    output wire        m_udp_payload_axis_tuser,

    // one-cycle pulse per packet: dropped because the FIFO was full / written whole
    output wire        stat_drop,
    output wire        stat_good
);

// ---------------------------------------------------------------- write side
localparam [1:0] W_IDLE = 2'd0, W_HDR0 = 2'd1, W_HDR1 = 2'd2, W_PAY = 2'd3;
reg [1:0]  wstate = W_IDLE;
reg        wdrop = 1'b0;
reg [63:0] hdr0 = 64'd0, hdr1 = 64'd0;

wire [31:0] alias_off = s_udp_ip_dest_ip - ALIAS_BASE;
wire ours  = (s_udp_ip_dest_ip == local_ip) || (alias_off < ALIAS_COUNT);
wire match = ours && ((s_udp_dest_port == PORT_A) || (s_udp_dest_port == PORT_B));

reg  [DATA_WIDTH-1:0] fifo_s_tdata;
reg  [KEEP_WIDTH-1:0] fifo_s_tkeep;
reg         fifo_s_tvalid, fifo_s_tlast, fifo_s_tuser;

assign s_udp_hdr_ready           = (wstate == W_IDLE);
assign s_udp_payload_axis_tready = (wstate == W_PAY);   // FIFO input is always ready

always @* begin
    fifo_s_tdata  = s_udp_payload_axis_tdata;
    fifo_s_tkeep  = s_udp_payload_axis_tkeep;
    fifo_s_tvalid = 1'b0;
    fifo_s_tlast  = s_udp_payload_axis_tlast;
    fifo_s_tuser  = s_udp_payload_axis_tuser;
    case (wstate)
        W_HDR0: begin fifo_s_tdata = hdr0; fifo_s_tkeep = {KEEP_WIDTH{1'b1}}; fifo_s_tvalid = 1'b1; fifo_s_tlast = 1'b0; fifo_s_tuser = 1'b0; end
        W_HDR1: begin fifo_s_tdata = hdr1; fifo_s_tkeep = {KEEP_WIDTH{1'b1}}; fifo_s_tvalid = 1'b1; fifo_s_tlast = 1'b0; fifo_s_tuser = 1'b0; end
        W_PAY:  fifo_s_tvalid = s_udp_payload_axis_tvalid && !wdrop;
        default: ;
    endcase
end

always @(posedge clk) begin
    if (rst) begin
        wstate <= W_IDLE;
    end else begin
        case (wstate)
        W_IDLE: if (s_udp_hdr_valid) begin
            hdr0  <= {s_udp_ip_source_ip, s_udp_source_port, s_udp_dest_port};
            hdr1  <= {16'd0, s_udp_ip_dest_ip, s_udp_length};
            wdrop <= !match;
            wstate <= match ? W_HDR0 : W_PAY;
        end
        W_HDR0: wstate <= W_HDR1;
        W_HDR1: wstate <= W_PAY;
        W_PAY:  if (s_udp_payload_axis_tvalid && s_udp_payload_axis_tlast) wstate <= W_IDLE;
        default: wstate <= W_IDLE;
        endcase
    end
end

// ---------------------------------------------------------------- frame FIFO
wire [DATA_WIDTH-1:0] fifo_m_tdata;
wire [KEEP_WIDTH-1:0] fifo_m_tkeep;
wire        fifo_m_tvalid, fifo_m_tlast, fifo_m_tuser;
wire        fifo_m_tready;

axis_fifo #(
    .DEPTH(FIFO_DEPTH),
    .DATA_WIDTH(DATA_WIDTH),
    .KEEP_ENABLE(1),
    .KEEP_WIDTH(KEEP_WIDTH),
    .LAST_ENABLE(1),
    .ID_ENABLE(0),
    .DEST_ENABLE(0),
    .USER_ENABLE(1),
    .USER_WIDTH(1),
    .FRAME_FIFO(1),
    .USER_BAD_FRAME_VALUE(1'b1),
    .USER_BAD_FRAME_MASK(1'b1),
    .DROP_OVERSIZE_FRAME(1),
    .DROP_BAD_FRAME(1),
    .DROP_WHEN_FULL(1)
)
frame_fifo (
    .clk(clk), .rst(rst),
    .s_axis_tdata(fifo_s_tdata), .s_axis_tkeep(fifo_s_tkeep), .s_axis_tvalid(fifo_s_tvalid),
    .s_axis_tready(), .s_axis_tlast(fifo_s_tlast), .s_axis_tid(8'd0), .s_axis_tdest(8'd0),
    .s_axis_tuser(fifo_s_tuser),
    .m_axis_tdata(fifo_m_tdata), .m_axis_tkeep(fifo_m_tkeep), .m_axis_tvalid(fifo_m_tvalid),
    .m_axis_tready(fifo_m_tready), .m_axis_tlast(fifo_m_tlast), .m_axis_tid(), .m_axis_tdest(),
    .m_axis_tuser(fifo_m_tuser),
    .status_overflow(stat_drop), .status_bad_frame(), .status_good_frame(stat_good)
);

// ---------------------------------------------------------------- read side
localparam [1:0] R_HDR0 = 2'd0, R_HDR1 = 2'd1, R_SEND = 2'd2, R_PAY = 2'd3;
reg [1:0]  rstate = R_HDR0;
reg [31:0] peer_ip = 32'd0;
reg [15:0] peer_port = 16'd0, our_port = 16'd0, udp_len = 16'd0;
reg [31:0] reply_ip = 32'd0;

assign fifo_m_tready = (rstate == R_HDR0) || (rstate == R_HDR1) ||
                       (rstate == R_PAY && m_udp_payload_axis_tready);

assign m_udp_hdr_valid    = (rstate == R_SEND);
assign m_udp_ip_dscp      = 6'd0;
assign m_udp_ip_ecn       = 2'd0;
assign m_udp_ip_ttl       = 8'd64;
assign m_udp_ip_source_ip = reply_ip;
assign m_udp_ip_dest_ip   = peer_ip;
assign m_udp_source_port  = our_port;
assign m_udp_dest_port    = peer_port;
assign m_udp_length       = udp_len;
assign m_udp_checksum     = 16'd0;            // 0 = no checksum (filled in by the stack if it generates one)

assign m_udp_payload_axis_tdata  = fifo_m_tdata;
assign m_udp_payload_axis_tkeep  = fifo_m_tkeep;
assign m_udp_payload_axis_tvalid = (rstate == R_PAY) && fifo_m_tvalid;
assign m_udp_payload_axis_tlast  = fifo_m_tlast;
assign m_udp_payload_axis_tuser  = fifo_m_tuser;

always @(posedge clk) begin
    if (rst) begin
        rstate <= R_HDR0;
    end else begin
        case (rstate)
        R_HDR0: if (fifo_m_tvalid) begin
            {peer_ip, peer_port, our_port} <= fifo_m_tdata[63:0];
            rstate <= R_HDR1;
        end
        R_HDR1: if (fifo_m_tvalid) begin
            udp_len <= fifo_m_tdata[15:0];
            reply_ip <= fifo_m_tdata[47:16];
            rstate <= R_SEND;
        end
        R_SEND: if (m_udp_hdr_ready) rstate <= R_PAY;
        R_PAY:  if (fifo_m_tvalid && m_udp_payload_axis_tready && fifo_m_tlast) rstate <= R_HDR0;
        default: rstate <= R_HDR0;
        endcase
    end
end

endmodule

`resetall
