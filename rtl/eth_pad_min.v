// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Yijie Yu
//
// Zero-pad Ethernet frames shorter than MIN_BYTES (60 = 64-byte minimum without FCS)
// on a wide AXI stream in front of the CMAC, which neither pads nor accepts runt frames.
// Requires KEEP_WIDTH >= MIN_BYTES, so a short frame is always a single beat.
// Combinational: no added latency.

`resetall
`timescale 1ns / 1ps
`default_nettype none

module eth_pad_min #(
    parameter DATA_WIDTH = 512,
    parameter KEEP_WIDTH = DATA_WIDTH / 8,
    parameter MIN_BYTES  = 60
)(
    input  wire                  clk,
    input  wire                  rst,

    input  wire [DATA_WIDTH-1:0] s_axis_tdata,
    input  wire [KEEP_WIDTH-1:0] s_axis_tkeep,
    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,
    input  wire                  s_axis_tlast,
    input  wire                  s_axis_tuser,

    output wire [DATA_WIDTH-1:0] m_axis_tdata,
    output wire [KEEP_WIDTH-1:0] m_axis_tkeep,
    output wire                  m_axis_tvalid,
    input  wire                  m_axis_tready,
    output wire                  m_axis_tlast,
    output wire                  m_axis_tuser
);

initial begin
    if (KEEP_WIDTH < MIN_BYTES) begin
        $error("eth_pad_min: KEEP_WIDTH must be >= MIN_BYTES");
        $finish;
    end
end

reg first_beat = 1'b1;
always @(posedge clk) begin
    if (rst)
        first_beat <= 1'b1;
    else if (s_axis_tvalid && s_axis_tready)
        first_beat <= s_axis_tlast;
end

// tkeep is contiguous from bit 0, so the frame is short when byte MIN_BYTES-1 is absent
wire short_frame = first_beat && s_axis_tlast && !s_axis_tkeep[MIN_BYTES-1];

wire [KEEP_WIDTH-1:0] min_keep = {{(KEEP_WIDTH-MIN_BYTES){1'b0}}, {MIN_BYTES{1'b1}}};

genvar i;
generate
    for (i = 0; i < KEEP_WIDTH; i = i + 1) begin : g_byte
        assign m_axis_tdata[i*8 +: 8] = (short_frame && !s_axis_tkeep[i]) ? 8'd0 : s_axis_tdata[i*8 +: 8];
    end
endgenerate

assign m_axis_tkeep  = short_frame ? (s_axis_tkeep | min_keep) : s_axis_tkeep;
assign m_axis_tvalid = s_axis_tvalid;
assign s_axis_tready = m_axis_tready;
assign m_axis_tlast  = s_axis_tlast;
assign m_axis_tuser  = s_axis_tuser;

endmodule

`resetall
