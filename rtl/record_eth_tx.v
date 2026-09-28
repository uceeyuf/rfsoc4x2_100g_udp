// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Yijie Yu
//
// Record stream as complete Ethernet/IPv4/UDP frames, 512-bit, one beat per cycle, no gap.
//
// The first beat of every frame holds all headers plus the record header, so the data beats
// from DDR go out unshifted:
//   bytes  0..13  Ethernet (dst MAC, src MAC, 0x0800)
//   bytes 14..33  IPv4 (DF, TTL 64, UDP, identification 0, header checksum)
//   bytes 34..41  UDP (checksum 0)
//   bytes 42..45  start_index    (little endian, index of the first sample in this packet)
//   bytes 46..49  TOTAL_SAMPLES  (little endian)
//   bytes 50..63  0
// followed by DATA_BEATS_STD or DATA_BEATS_JUMBO beats of samples (16 x 32 bit per beat).
// UDP payload = 22 + 64 * beats bytes (1046 standard, 8214 jumbo).
//
// Flow k (packet n uses flow n mod flow_count) adds k to both ports and, with flow_vary_ip,
// to the source IP. start_index runs over all flows.

`resetall
`timescale 1ns / 1ps
`default_nettype none

module record_eth_tx #(
    parameter TOTAL_SAMPLES    = 1048576,   // samples per record, multiple of 16 * beats
    parameter DATA_BEATS_STD   = 16,
    parameter DATA_BEATS_JUMBO = 128,
    parameter LOOP             = 1
)(
    input  wire         clk,
    input  wire         rst,

    input  wire [47:0]  local_mac,
    input  wire [31:0]  local_ip,
    input  wire [15:0]  local_port,
    input  wire [47:0]  dest_mac,
    input  wire [31:0]  dest_ip,
    input  wire [15:0]  dest_port,

    input  wire         trig,               // level: stream while high (stops after the current frame)
    input  wire [15:0]  gap_cycles,         // idle cycles between frames
    input  wire [4:0]   flow_count,         // 0/1 = one flow
    input  wire         flow_vary_ip,
    input  wire         jumbo,              // latched when trig rises
    output reg  [3:0]   txstate,

    // sample data, 16 samples per beat
    input  wire [511:0] s_axis_tdata,
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,

    // Ethernet frames (without FCS)
    output wire [511:0] m_axis_tdata,
    output wire [63:0]  m_axis_tkeep,
    output wire         m_axis_tvalid,
    input  wire         m_axis_tready,
    output wire         m_axis_tlast,
    output wire         m_axis_tuser
);

localparam [3:0] S_IDLE = 4'd0, S_PREP = 4'd1, S_FRAME = 4'd2, S_GAP = 4'd3, S_DONE = 4'd4;
localparam [31:0] SPB = 16;             // samples per beat

reg        trig_d = 1'b0;
reg [4:0]  flow_idx = 5'd0;
reg [7:0]  beats = DATA_BEATS_STD;      // data beats per frame
reg [7:0]  bcnt = 8'd0;                 // 0 = header beat, 1..beats = data
reg [31:0] beat_s = 32'd0;              // index of the next sample to send
reg [31:0] start_index = 32'd0;
reg [15:0] gapc = 16'd0;
reg [2:0]  prep = 3'd0;

wire [4:0] flow_last = (flow_count == 5'd0) ? 5'd0 : flow_count - 5'd1;

// ---------------------------------------------------------------- header fields (for flow_idx)
reg [31:0] src_ip = 32'd0;
reg [15:0] src_port = 16'd0, dst_port = 16'd0, ip_len = 16'd0, udp_len = 16'd0;
reg [19:0] csum_a = 20'd0;
reg [15:0] ip_csum = 16'd0;
wire [16:0] csum_f = {1'b0, csum_a[15:0]} + {13'd0, csum_a[19:16]};

// Three register stages; flow_idx changes at a header beat, and a frame is at least
// 17 beats long, so the next header is always settled in time. S_PREP covers the first one.
always @(posedge clk) begin
    src_ip   <= local_ip + (flow_vary_ip ? {27'd0, flow_idx} : 32'd0);
    src_port <= local_port + {11'd0, flow_idx};
    dst_port <= dest_port  + {11'd0, flow_idx};
    udp_len  <= 16'd8 + 16'd22 + {beats, 6'd0};
    ip_len   <= 16'd28 + 16'd22 + {beats, 6'd0};

    csum_a   <= 20'h4500 + ip_len + 20'h4000 + 20'h4011
              + src_ip[31:16] + src_ip[15:0] + dest_ip[31:16] + dest_ip[15:0];
    ip_csum  <= ~(csum_f[15:0] + {15'd0, csum_f[16]});
end

// byte i of the frame is tdata[8*i +: 8]; multi-byte header fields are big endian
function [15:0] be16(input [15:0] v); be16 = {v[7:0], v[15:8]}; endfunction
function [31:0] be32(input [31:0] v); be32 = {v[7:0], v[15:8], v[23:16], v[31:24]}; endfunction
function [47:0] be48(input [47:0] v); be48 = {v[7:0], v[15:8], v[23:16], v[31:24], v[39:32], v[47:40]}; endfunction

wire [511:0] hdr_beat = {
    112'd0,                         // 50..63
    TOTAL_SAMPLES[31:0],            // 46..49
    start_index,                    // 42..45
    16'd0,                          // 40..41 UDP checksum
    be16(udp_len),                  // 38..39
    be16(dst_port),                 // 36..37
    be16(src_port),                 // 34..35
    be32(dest_ip),                  // 30..33
    be32(src_ip),                   // 26..29
    be16(ip_csum),                  // 24..25
    8'h11, 8'd64,                   // 23 protocol, 22 TTL
    16'h0040,                       // 20..21 flags DF, fragment offset 0
    16'd0,                          // 18..19 identification
    be16(ip_len),                   // 16..17
    8'h00, 8'h45,                   // 15 DSCP/ECN, 14 version/IHL
    16'h0008,                       // 12..13 EtherType 0x0800
    be48(local_mac),                // 6..11
    be48(dest_mac)                  // 0..5
};

// ---------------------------------------------------------------- frame output
wire in_frame = (txstate == S_FRAME);
wire is_hdr   = (bcnt == 8'd0);

assign m_axis_tdata  = is_hdr ? hdr_beat : s_axis_tdata;
assign m_axis_tkeep  = {64{1'b1}};
assign m_axis_tvalid = in_frame & (is_hdr | s_axis_tvalid);
assign m_axis_tlast  = (bcnt == beats);
assign m_axis_tuser  = 1'b0;
assign s_axis_tready = in_frame & ~is_hdr & m_axis_tready;

wire fire = m_axis_tvalid & m_axis_tready;
wire [31:0] beat_s_next = (beat_s + SPB >= TOTAL_SAMPLES) ? 32'd0 : beat_s + SPB;

always @(posedge clk) begin
    trig_d <= trig;
    if (rst) begin
        txstate <= S_IDLE;
        trig_d <= 1'b0;
        flow_idx <= 5'd0;
        bcnt <= 8'd0;
        beat_s <= 32'd0;
        start_index <= 32'd0;
    end else begin
        case (txstate)
        S_IDLE: begin
            if (trig && !trig_d && dest_ip != 32'd0) begin
                beats <= jumbo ? DATA_BEATS_JUMBO : DATA_BEATS_STD;
                flow_idx <= 5'd0;
                beat_s <= 32'd0;
                start_index <= 32'd0;
                bcnt <= 8'd0;
                prep <= 3'd0;
                txstate <= S_PREP;
            end
        end
        S_PREP: begin   // let the header registers settle for flow 0
            prep <= prep + 3'd1;
            if (prep == 3'd7)
                txstate <= S_FRAME;
        end
        S_FRAME: begin
            if (fire) begin
                if (is_hdr) begin
                    // header registers now follow the next flow
                    flow_idx <= (flow_idx >= flow_last) ? 5'd0 : flow_idx + 5'd1;
                end else begin
                    beat_s <= beat_s_next;
                end
                if (m_axis_tlast) begin
                    bcnt <= 8'd0;
                    start_index <= beat_s_next;
                    gapc <= 16'd0;
                    if (!trig || (!LOOP && beat_s_next == 32'd0))
                        txstate <= S_DONE;
                    else if (gap_cycles != 16'd0)
                        txstate <= S_GAP;
                end else begin
                    bcnt <= bcnt + 8'd1;
                end
            end
        end
        S_GAP: begin
            gapc <= gapc + 16'd1;
            if (gapc + 16'd1 >= gap_cycles)
                txstate <= trig ? S_FRAME : S_DONE;
        end
        S_DONE: begin
            if (!trig)
                txstate <= S_IDLE;
        end
        default: txstate <= S_IDLE;
        endcase
    end
end

endmodule

`resetall
