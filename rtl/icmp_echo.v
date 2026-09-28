// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Yijie Yu
//
// ICMP echo responder on the IP interface of udp_complete_64.
// An ICMP echo request (type 8) addressed to local_ip is answered with an echo reply
// (type 0) carrying the same identifier, sequence number and data. Only the type byte
// changes, so the ICMP checksum is updated incrementally (RFC 1624).
//
// Like udp_echo, the receive side never waits for the transmit side: requests go into a
// frame FIFO as one header word {source IP, 16'd0, IP length} plus the reply payload;
// anything that is not an echo request is marked bad and dropped by the FIFO.

`resetall
`timescale 1ns / 1ps
`default_nettype none

module icmp_echo #(
    parameter DATA_WIDTH = 64,
    parameter KEEP_WIDTH = DATA_WIDTH / 8,
    parameter FIFO_DEPTH = 16384            // bytes
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] local_ip,

    // received IP packets (non-UDP)
    input  wire        s_ip_hdr_valid,
    output wire        s_ip_hdr_ready,
    input  wire [15:0] s_ip_length,
    input  wire [7:0]  s_ip_protocol,
    input  wire [31:0] s_ip_source_ip,
    input  wire [31:0] s_ip_dest_ip,
    input  wire [DATA_WIDTH-1:0] s_ip_payload_axis_tdata,
    input  wire [KEEP_WIDTH-1:0] s_ip_payload_axis_tkeep,
    input  wire        s_ip_payload_axis_tvalid,
    output wire        s_ip_payload_axis_tready,
    input  wire        s_ip_payload_axis_tlast,
    input  wire        s_ip_payload_axis_tuser,

    // IP packets to transmit
    output wire        m_ip_hdr_valid,
    input  wire        m_ip_hdr_ready,
    output wire [5:0]  m_ip_dscp,
    output wire [1:0]  m_ip_ecn,
    output wire [15:0] m_ip_length,
    output wire [7:0]  m_ip_ttl,
    output wire [7:0]  m_ip_protocol,
    output wire [31:0] m_ip_source_ip,
    output wire [31:0] m_ip_dest_ip,
    output wire [DATA_WIDTH-1:0] m_ip_payload_axis_tdata,
    output wire [KEEP_WIDTH-1:0] m_ip_payload_axis_tkeep,
    output wire        m_ip_payload_axis_tvalid,
    input  wire        m_ip_payload_axis_tready,
    output wire        m_ip_payload_axis_tlast,
    output wire        m_ip_payload_axis_tuser
);

// ---------------------------------------------------------------- write side
localparam [1:0] W_IDLE = 2'd0, W_HDR = 2'd1, W_PAY = 2'd2;
reg [1:0]  wstate = W_IDLE;
reg        wdrop = 1'b0;      // not ICMP to us: consume without writing
reg        wfirst = 1'b0;     // next payload word is the first (ICMP header)
reg        wbad = 1'b0;       // not an echo request: drop in the FIFO
reg [63:0] hdr = 64'd0;

// first payload word: bytes 0..3 = type, code, checksum (big endian)
wire [7:0]  icmp_type = s_ip_payload_axis_tdata[7:0];
wire        is_request = (icmp_type == 8'd8) && (s_ip_payload_axis_tkeep[3:0] == 4'hf);
wire [15:0] csum_in   = {s_ip_payload_axis_tdata[23:16], s_ip_payload_axis_tdata[31:24]};
// RFC 1624 eqn. 3 with m = 0x0800 (type 8, code 0) -> m' = 0x0000 (type 0, code 0):
// HC' = ~(~HC + ~m + m') = ~(~HC + 0xF7FF), one's complement addition
wire [16:0] csum_s1   = {1'b0, ~csum_in} + 17'h0F7FF;
wire [16:0] csum_s2   = {1'b0, csum_s1[15:0]} + {16'd0, csum_s1[16]};
wire [15:0] csum_out  = ~(csum_s2[15:0] + {15'd0, csum_s2[16]});
wire [DATA_WIDTH-1:0] reply_first = {s_ip_payload_axis_tdata[DATA_WIDTH-1:32], csum_out[7:0], csum_out[15:8],
                           s_ip_payload_axis_tdata[15:8], 8'h00};

assign s_ip_hdr_ready           = (wstate == W_IDLE);
assign s_ip_payload_axis_tready = (wstate == W_PAY);    // FIFO input is always ready

wire   bad_now = wbad || (wfirst && !is_request);

wire [DATA_WIDTH-1:0] fifo_s_tdata  = (wstate == W_HDR) ? {{(DATA_WIDTH-64){1'b0}}, hdr} : (wfirst ? reply_first : s_ip_payload_axis_tdata);
wire [KEEP_WIDTH-1:0] fifo_s_tkeep  = (wstate == W_HDR) ? {KEEP_WIDTH{1'b1}} : s_ip_payload_axis_tkeep;
wire        fifo_s_tvalid = (wstate == W_HDR) || (wstate == W_PAY && s_ip_payload_axis_tvalid && !wdrop);
wire        fifo_s_tlast  = (wstate == W_PAY) && s_ip_payload_axis_tlast;
wire        fifo_s_tuser  = (wstate == W_PAY) && (s_ip_payload_axis_tuser || bad_now);

always @(posedge clk) begin
    if (rst) begin
        wstate <= W_IDLE;
    end else begin
        case (wstate)
        W_IDLE: if (s_ip_hdr_valid) begin
            hdr    <= {s_ip_source_ip, 16'd0, s_ip_length};
            wdrop  <= !(s_ip_protocol == 8'd1 && s_ip_dest_ip == local_ip);
            wfirst <= 1'b1;
            wbad   <= 1'b0;
            wstate <= (s_ip_protocol == 8'd1 && s_ip_dest_ip == local_ip) ? W_HDR : W_PAY;
        end
        W_HDR: wstate <= W_PAY;
        W_PAY: if (s_ip_payload_axis_tvalid) begin
            wfirst <= 1'b0;
            wbad   <= bad_now;
            if (s_ip_payload_axis_tlast) wstate <= W_IDLE;
        end
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
    .status_overflow(), .status_bad_frame(), .status_good_frame()
);

// ---------------------------------------------------------------- read side
localparam [1:0] R_HDR = 2'd0, R_SEND = 2'd1, R_PAY = 2'd2;
reg [1:0]  rstate = R_HDR;
reg [31:0] peer_ip = 32'd0;
reg [15:0] ip_len = 16'd0;

assign fifo_m_tready = (rstate == R_HDR) || (rstate == R_PAY && m_ip_payload_axis_tready);

assign m_ip_hdr_valid = (rstate == R_SEND);
assign m_ip_dscp      = 6'd0;
assign m_ip_ecn       = 2'd0;
assign m_ip_length    = ip_len;
assign m_ip_ttl       = 8'd64;
assign m_ip_protocol  = 8'd1;
assign m_ip_source_ip = local_ip;
assign m_ip_dest_ip   = peer_ip;

assign m_ip_payload_axis_tdata  = fifo_m_tdata;
assign m_ip_payload_axis_tkeep  = fifo_m_tkeep;
assign m_ip_payload_axis_tvalid = (rstate == R_PAY) && fifo_m_tvalid;
assign m_ip_payload_axis_tlast  = fifo_m_tlast;
assign m_ip_payload_axis_tuser  = fifo_m_tuser;

always @(posedge clk) begin
    if (rst) begin
        rstate <= R_HDR;
    end else begin
        case (rstate)
        R_HDR: if (fifo_m_tvalid) begin
            peer_ip <= fifo_m_tdata[63:32];
            ip_len  <= fifo_m_tdata[15:0];
            rstate  <= R_SEND;
        end
        R_SEND: if (m_ip_hdr_ready) rstate <= R_PAY;
        R_PAY:  if (fifo_m_tvalid && m_ip_payload_axis_tready && fifo_m_tlast) rstate <= R_HDR;
        default: rstate <= R_HDR;
        endcase
    end
end

endmodule

`resetall
