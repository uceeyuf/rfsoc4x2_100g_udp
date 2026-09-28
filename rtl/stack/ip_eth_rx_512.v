/*

Copyright (c) 2014-2018 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * IP ethernet frame receiver (Ethernet frame in, IP frame out, wide datapath)
 *
 * Parameterized rewrite of ip_eth_rx_64 for wide buses where the whole 20-byte
 * IPv4 header (no options, IHL=5) lands in the first payload beat.
 * Requires KEEP_WIDTH > 20 (e.g. 256/512-bit). Only IHL=5 is accepted.
 */
module ip_eth_rx_512 #
(
    parameter DATA_WIDTH = 512,
    parameter KEEP_ENABLE = (DATA_WIDTH>8),
    parameter KEEP_WIDTH = (DATA_WIDTH/8)
)
(
    input  wire                   clk,
    input  wire                   rst,

    /*
     * Ethernet frame input
     */
    input  wire                   s_eth_hdr_valid,
    output wire                   s_eth_hdr_ready,
    input  wire [47:0]            s_eth_dest_mac,
    input  wire [47:0]            s_eth_src_mac,
    input  wire [15:0]            s_eth_type,
    input  wire [DATA_WIDTH-1:0]  s_eth_payload_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]  s_eth_payload_axis_tkeep,
    input  wire                   s_eth_payload_axis_tvalid,
    output wire                   s_eth_payload_axis_tready,
    input  wire                   s_eth_payload_axis_tlast,
    input  wire                   s_eth_payload_axis_tuser,

    /*
     * IP frame output
     */
    output wire                   m_ip_hdr_valid,
    input  wire                   m_ip_hdr_ready,
    output wire [47:0]            m_eth_dest_mac,
    output wire [47:0]            m_eth_src_mac,
    output wire [15:0]            m_eth_type,
    output wire [3:0]             m_ip_version,
    output wire [3:0]             m_ip_ihl,
    output wire [5:0]             m_ip_dscp,
    output wire [1:0]             m_ip_ecn,
    output wire [15:0]            m_ip_length,
    output wire [15:0]            m_ip_identification,
    output wire [2:0]             m_ip_flags,
    output wire [12:0]            m_ip_fragment_offset,
    output wire [7:0]             m_ip_ttl,
    output wire [7:0]             m_ip_protocol,
    output wire [15:0]            m_ip_header_checksum,
    output wire [31:0]            m_ip_source_ip,
    output wire [31:0]            m_ip_dest_ip,
    output wire [DATA_WIDTH-1:0]  m_ip_payload_axis_tdata,
    output wire [KEEP_WIDTH-1:0]  m_ip_payload_axis_tkeep,
    output wire                   m_ip_payload_axis_tvalid,
    input  wire                   m_ip_payload_axis_tready,
    output wire                   m_ip_payload_axis_tlast,
    output wire                   m_ip_payload_axis_tuser,

    /*
     * Status signals
     */
    output wire                   busy,
    output wire                   error_header_early_termination,
    output wire                   error_payload_early_termination,
    output wire                   error_invalid_header,
    output wire                   error_invalid_checksum
);

// IPv4 header length (no options)
localparam HDR_LEN = 20;
// payload byte offset within the first beat = header length (header occupies low HDR_LEN bytes)
localparam OFFSET = HDR_LEN;
// number of low bytes of each output beat taken from the saved (previous) input beat
localparam LOW = KEEP_WIDTH - OFFSET;

localparam CL = $clog2(KEEP_WIDTH+1);

// bus width assertions
initial begin
    if (KEEP_WIDTH * 8 != DATA_WIDTH) begin
        $error("Error: AXI stream interface requires byte (8-bit) granularity (instance %m)");
        $finish;
    end
    if (KEEP_WIDTH <= HDR_LEN) begin
        $error("Error: ip_eth_rx_512 requires KEEP_WIDTH > 20 (whole header in first beat) (instance %m)");
        $finish;
    end
end

localparam [1:0]
    STATE_IDLE = 2'd0,
    STATE_READ_HEADER = 2'd1,
    STATE_READ_PAYLOAD = 2'd2,
    STATE_WAIT_LAST = 2'd3;

reg [1:0] state_reg = STATE_IDLE, state_next;

// datapath control signals
reg store_eth_hdr;
reg store_hdr;
reg flush_save;
reg transfer_in_save;

reg [15:0] word_count_reg = 16'd0, word_count_next;

reg s_eth_hdr_ready_reg = 1'b0, s_eth_hdr_ready_next;
reg s_eth_payload_axis_tready_reg = 1'b0, s_eth_payload_axis_tready_next;

reg m_ip_hdr_valid_reg = 1'b0, m_ip_hdr_valid_next;
reg [47:0] m_eth_dest_mac_reg = 48'd0;
reg [47:0] m_eth_src_mac_reg = 48'd0;
reg [15:0] m_eth_type_reg = 16'd0;
reg [3:0]  m_ip_version_reg = 4'd0;
reg [3:0]  m_ip_ihl_reg = 4'd0;
reg [5:0]  m_ip_dscp_reg = 6'd0;
reg [1:0]  m_ip_ecn_reg = 2'd0;
reg [15:0] m_ip_length_reg = 16'd0;
reg [15:0] m_ip_identification_reg = 16'd0;
reg [2:0]  m_ip_flags_reg = 3'd0;
reg [12:0] m_ip_fragment_offset_reg = 13'd0;
reg [7:0]  m_ip_ttl_reg = 8'd0;
reg [7:0]  m_ip_protocol_reg = 8'd0;
reg [15:0] m_ip_header_checksum_reg = 16'd0;
reg [31:0] m_ip_source_ip_reg = 32'd0;
reg [31:0] m_ip_dest_ip_reg = 32'd0;

reg busy_reg = 1'b0;
reg error_header_early_termination_reg = 1'b0, error_header_early_termination_next;
reg error_payload_early_termination_reg = 1'b0, error_payload_early_termination_next;
reg error_invalid_header_reg = 1'b0, error_invalid_header_next;
reg error_invalid_checksum_reg = 1'b0, error_invalid_checksum_next;

// saved previous input beat (for realignment)
reg [DATA_WIDTH-1:0] save_data_reg = {DATA_WIDTH{1'b0}};
reg [KEEP_WIDTH-1:0] save_keep_reg = {KEEP_WIDTH{1'b0}};
reg save_last_reg = 1'b0;
reg save_user_reg = 1'b0;
reg extra_cycle_reg = 1'b0;

// shifted (realigned) payload word
reg [DATA_WIDTH-1:0] shift_data;
reg [KEEP_WIDTH-1:0] shift_keep;
reg shift_valid;
reg shift_last;
reg shift_user;
reg shift_s_tready;

// internal datapath
reg [DATA_WIDTH-1:0] m_ip_payload_axis_tdata_int;
reg [KEEP_WIDTH-1:0] m_ip_payload_axis_tkeep_int;
reg                  m_ip_payload_axis_tvalid_int;
reg                  m_ip_payload_axis_tready_int_reg = 1'b0;
reg                  m_ip_payload_axis_tlast_int;
reg                  m_ip_payload_axis_tuser_int;
wire                 m_ip_payload_axis_tready_int_early;

assign s_eth_hdr_ready = s_eth_hdr_ready_reg;

// input register stage (skid buffer, full throughput): the IPv4 header checksum of each beat
// is summed on its way in and travels with the beat, so the state machine only sees the
// registered ok bit instead of a 10-word adder tree
wire [DATA_WIDTH-1:0] in_tdata;
wire [KEEP_WIDTH-1:0] in_tkeep;
wire                  in_tvalid, in_tlast, in_tuser, in_csum_ok;
wire                  s_csum_ok;

axis_register #(
    .DATA_WIDTH(DATA_WIDTH), .KEEP_ENABLE(1), .KEEP_WIDTH(KEEP_WIDTH), .LAST_ENABLE(1),
    .ID_ENABLE(0), .DEST_ENABLE(0), .USER_ENABLE(1), .USER_WIDTH(2), .REG_TYPE(2)
)
in_reg (
    .clk(clk), .rst(rst),
    .s_axis_tdata(s_eth_payload_axis_tdata), .s_axis_tkeep(s_eth_payload_axis_tkeep),
    .s_axis_tvalid(s_eth_payload_axis_tvalid), .s_axis_tready(s_eth_payload_axis_tready),
    .s_axis_tlast(s_eth_payload_axis_tlast), .s_axis_tid(8'd0), .s_axis_tdest(8'd0),
    .s_axis_tuser({s_csum_ok, s_eth_payload_axis_tuser}),
    .m_axis_tdata(in_tdata), .m_axis_tkeep(in_tkeep), .m_axis_tvalid(in_tvalid),
    .m_axis_tready(s_eth_payload_axis_tready_reg), .m_axis_tlast(in_tlast), .m_axis_tid(), .m_axis_tdest(),
    .m_axis_tuser({in_csum_ok, in_tuser})
);

assign m_ip_hdr_valid = m_ip_hdr_valid_reg;
assign m_eth_dest_mac = m_eth_dest_mac_reg;
assign m_eth_src_mac = m_eth_src_mac_reg;
assign m_eth_type = m_eth_type_reg;
assign m_ip_version = m_ip_version_reg;
assign m_ip_ihl = m_ip_ihl_reg;
assign m_ip_dscp = m_ip_dscp_reg;
assign m_ip_ecn = m_ip_ecn_reg;
assign m_ip_length = m_ip_length_reg;
assign m_ip_identification = m_ip_identification_reg;
assign m_ip_flags = m_ip_flags_reg;
assign m_ip_fragment_offset = m_ip_fragment_offset_reg;
assign m_ip_ttl = m_ip_ttl_reg;
assign m_ip_protocol = m_ip_protocol_reg;
assign m_ip_header_checksum = m_ip_header_checksum_reg;
assign m_ip_source_ip = m_ip_source_ip_reg;
assign m_ip_dest_ip = m_ip_dest_ip_reg;

assign busy = busy_reg;
assign error_header_early_termination = error_header_early_termination_reg;
assign error_payload_early_termination = error_payload_early_termination_reg;
assign error_invalid_header = error_invalid_header_reg;
assign error_invalid_checksum = error_invalid_checksum_reg;

// byte helper for the first (header) beat
function [7:0] hbyte;
    input [DATA_WIDTH-1:0] d;
    input integer i;
    hbyte = d[i*8 +: 8];
endfunction

// header field decode (combinational, from current input beat)
wire [3:0]  hdr_version = in_tdata[7:4];
wire [3:0]  hdr_ihl     = in_tdata[3:0];
wire [5:0]  hdr_dscp    = in_tdata[15:10];
wire [1:0]  hdr_ecn     = in_tdata[9:8];
wire [15:0] hdr_length  = {hbyte(in_tdata, 2),  hbyte(in_tdata, 3)};
wire [15:0] hdr_ident   = {hbyte(in_tdata, 4),  hbyte(in_tdata, 5)};
wire [2:0]  hdr_flags   = in_tdata[6*8+7 -:3];
wire [12:0] hdr_frag    = {in_tdata[6*8+4 -:5], hbyte(in_tdata, 7)};
wire [7:0]  hdr_ttl     = hbyte(in_tdata, 8);
wire [7:0]  hdr_proto   = hbyte(in_tdata, 9);
wire [15:0] hdr_csum    = {hbyte(in_tdata, 10), hbyte(in_tdata, 11)};
wire [31:0] hdr_src_ip  = {hbyte(in_tdata, 12), hbyte(in_tdata, 13), hbyte(in_tdata, 14), hbyte(in_tdata, 15)};
wire [31:0] hdr_dst_ip  = {hbyte(in_tdata, 16), hbyte(in_tdata, 17), hbyte(in_tdata, 18), hbyte(in_tdata, 19)};

// header checksum over the 10 16-bit words of the (option-less) header, of the beat entering in_reg
wire [19:0] hdr_sum =
    {hbyte(s_eth_payload_axis_tdata, 0),  hbyte(s_eth_payload_axis_tdata, 1)}  +
    {hbyte(s_eth_payload_axis_tdata, 2),  hbyte(s_eth_payload_axis_tdata, 3)}  +
    {hbyte(s_eth_payload_axis_tdata, 4),  hbyte(s_eth_payload_axis_tdata, 5)}  +
    {hbyte(s_eth_payload_axis_tdata, 6),  hbyte(s_eth_payload_axis_tdata, 7)}  +
    {hbyte(s_eth_payload_axis_tdata, 8),  hbyte(s_eth_payload_axis_tdata, 9)}  +
    {hbyte(s_eth_payload_axis_tdata, 10), hbyte(s_eth_payload_axis_tdata, 11)} +
    {hbyte(s_eth_payload_axis_tdata, 12), hbyte(s_eth_payload_axis_tdata, 13)} +
    {hbyte(s_eth_payload_axis_tdata, 14), hbyte(s_eth_payload_axis_tdata, 15)} +
    {hbyte(s_eth_payload_axis_tdata, 16), hbyte(s_eth_payload_axis_tdata, 17)} +
    {hbyte(s_eth_payload_axis_tdata, 18), hbyte(s_eth_payload_axis_tdata, 19)};
wire [16:0] hdr_sum_f1 = hdr_sum[15:0] + hdr_sum[19:16];
wire [15:0] hdr_sum_f2 = hdr_sum_f1[15:0] + hdr_sum_f1[16];
assign s_csum_ok = (hdr_sum_f2 == 16'hffff);
wire hdr_csum_ok = in_csum_ok;

function [KEEP_WIDTH-1:0] count2keep;
    input [CL-1:0] c;
    begin
        if (c >= KEEP_WIDTH)
            count2keep = {KEEP_WIDTH{1'b1}};
        else
            count2keep = {KEEP_WIDTH{1'b1}} >> (KEEP_WIDTH - c);
    end
endfunction

function [CL-1:0] keep2count;
    input [KEEP_WIDTH-1:0] k;
    integer i;
    begin
        keep2count = 0;
        for (i = 0; i < KEEP_WIDTH; i = i + 1)
            if (k[i]) keep2count = i+1;
    end
endfunction

// realignment shifter: output low LOW bytes from saved high, high OFFSET bytes from current low
always @* begin
    shift_data[0 +: LOW*8] = save_data_reg[OFFSET*8 +: LOW*8];
    shift_keep[0 +: LOW]   = save_keep_reg[OFFSET +: LOW];

    if (extra_cycle_reg) begin
        shift_data[LOW*8 +: OFFSET*8] = {(OFFSET*8){1'b0}};
        shift_keep[LOW +: OFFSET]     = {OFFSET{1'b0}};
        shift_valid = 1'b1;
        shift_last  = save_last_reg;
        shift_user  = save_user_reg;
        shift_s_tready = flush_save;
    end else begin
        shift_data[LOW*8 +: OFFSET*8] = in_tdata[0 +: OFFSET*8];
        shift_keep[LOW +: OFFSET]     = in_tkeep[0 +: OFFSET];
        shift_valid = in_tvalid;
        shift_last  = in_tlast && (in_tkeep[OFFSET +: LOW] == {LOW{1'b0}});
        shift_user  = in_tuser && (in_tkeep[OFFSET +: LOW] == {LOW{1'b0}});
        shift_s_tready = !(in_tlast && in_tvalid && transfer_in_save);
    end
end

always @* begin
    state_next = STATE_IDLE;

    flush_save = 1'b0;
    transfer_in_save = 1'b0;

    s_eth_hdr_ready_next = 1'b0;
    s_eth_payload_axis_tready_next = 1'b0;

    store_eth_hdr = 1'b0;
    store_hdr = 1'b0;

    word_count_next = word_count_reg;

    m_ip_hdr_valid_next = m_ip_hdr_valid_reg && !m_ip_hdr_ready;

    error_header_early_termination_next = 1'b0;
    error_payload_early_termination_next = 1'b0;
    error_invalid_header_next = 1'b0;
    error_invalid_checksum_next = 1'b0;

    m_ip_payload_axis_tdata_int = {DATA_WIDTH{1'b0}};
    m_ip_payload_axis_tkeep_int = {KEEP_WIDTH{1'b0}};
    m_ip_payload_axis_tvalid_int = 1'b0;
    m_ip_payload_axis_tlast_int = 1'b0;
    m_ip_payload_axis_tuser_int = 1'b0;

    case (state_reg)
        STATE_IDLE: begin
            // idle - wait for header
            flush_save = 1'b1;
            s_eth_hdr_ready_next = !m_ip_hdr_valid_next;

            if (s_eth_hdr_ready && s_eth_hdr_valid) begin
                s_eth_hdr_ready_next = 1'b0;
                s_eth_payload_axis_tready_next = 1'b1;
                store_eth_hdr = 1'b1;
                state_next = STATE_READ_HEADER;
            end else begin
                state_next = STATE_IDLE;
            end
        end
        STATE_READ_HEADER: begin
            // first payload beat carries the whole IP header (low 20 bytes)
            s_eth_payload_axis_tready_next = shift_s_tready;

            if (in_tvalid) begin
                // store header from this beat, push beat into save
                store_hdr = 1'b1;
                transfer_in_save = 1'b1;
                word_count_next = hdr_length - HDR_LEN;

                if (hdr_version != 4'd4 || hdr_ihl != 4'd5) begin
                    error_invalid_header_next = 1'b1;
                    s_eth_payload_axis_tready_next = shift_s_tready;
                    state_next = STATE_WAIT_LAST;
                end else if (!hdr_csum_ok) begin
                    error_invalid_checksum_next = 1'b1;
                    s_eth_payload_axis_tready_next = shift_s_tready;
                    state_next = STATE_WAIT_LAST;
                end else begin
                    m_ip_hdr_valid_next = 1'b1;
                    s_eth_payload_axis_tready_next = m_ip_payload_axis_tready_int_early && shift_s_tready;
                    state_next = STATE_READ_PAYLOAD;
                end

                if (in_tlast) begin
                    if (hdr_version != 4'd4 || hdr_ihl != 4'd5 || !hdr_csum_ok) begin
                        // header error already flagged; nothing more to do
                        s_eth_payload_axis_tready_next = 1'b0;
                        flush_save = 1'b1;
                        m_ip_hdr_valid_next = 1'b0;
                        s_eth_hdr_ready_next = 1'b1;
                        state_next = STATE_IDLE;
                    end
                    // else: payload (if any) flushed out of save in STATE_READ_PAYLOAD via extra cycle
                end
            end else begin
                state_next = STATE_READ_HEADER;
            end
        end
        STATE_READ_PAYLOAD: begin
            // realigned payload words
            s_eth_payload_axis_tready_next = m_ip_payload_axis_tready_int_early && shift_s_tready;

            m_ip_payload_axis_tdata_int = shift_data;
            m_ip_payload_axis_tkeep_int = shift_keep;
            m_ip_payload_axis_tlast_int = shift_last;
            m_ip_payload_axis_tuser_int = shift_user;

            if (m_ip_payload_axis_tready_int_reg && shift_valid) begin
                // word transfer through
                transfer_in_save = 1'b1;
                m_ip_payload_axis_tvalid_int = 1'b1;

                if (word_count_reg <= KEEP_WIDTH) begin
                    // last payload word (by IP length); mask padding
                    m_ip_payload_axis_tkeep_int = shift_keep & count2keep(word_count_reg[CL-1:0]);
                    m_ip_payload_axis_tlast_int = 1'b1;
                    word_count_next = 16'd0;

                    if (shift_last) begin
                        if (keep2count(shift_keep) < word_count_reg) begin
                            // frame ended early relative to IP length
                            error_payload_early_termination_next = 1'b1;
                            m_ip_payload_axis_tuser_int = 1'b1;
                        end
                        s_eth_payload_axis_tready_next = 1'b0;
                        flush_save = 1'b1;
                        s_eth_hdr_ready_next = !m_ip_hdr_valid_next;
                        state_next = STATE_IDLE;
                    end else begin
                        // payload complete but frame continues (eth padding) - drop the rest
                        s_eth_payload_axis_tready_next = shift_s_tready;
                        state_next = STATE_WAIT_LAST;
                    end
                end else begin
                    word_count_next = word_count_reg - KEEP_WIDTH;
                    if (shift_last) begin
                        // frame ended before IP length satisfied
                        error_payload_early_termination_next = 1'b1;
                        m_ip_payload_axis_tuser_int = 1'b1;
                        s_eth_payload_axis_tready_next = 1'b0;
                        flush_save = 1'b1;
                        s_eth_hdr_ready_next = !m_ip_hdr_valid_next;
                        state_next = STATE_IDLE;
                    end else begin
                        state_next = STATE_READ_PAYLOAD;
                    end
                end
            end else begin
                state_next = STATE_READ_PAYLOAD;
            end
        end
        STATE_WAIT_LAST: begin
            // drop remaining input until end of frame
            s_eth_payload_axis_tready_next = shift_s_tready;

            if (shift_valid) begin
                transfer_in_save = 1'b1;
                if (shift_last) begin
                    s_eth_payload_axis_tready_next = 1'b0;
                    flush_save = 1'b1;
                    s_eth_hdr_ready_next = !m_ip_hdr_valid_next;
                    state_next = STATE_IDLE;
                end else begin
                    state_next = STATE_WAIT_LAST;
                end
            end else begin
                state_next = STATE_WAIT_LAST;
            end
        end
    endcase
end

always @(posedge clk) begin
    state_reg <= state_next;

    s_eth_hdr_ready_reg <= s_eth_hdr_ready_next;
    s_eth_payload_axis_tready_reg <= s_eth_payload_axis_tready_next;

    m_ip_hdr_valid_reg <= m_ip_hdr_valid_next;

    word_count_reg <= word_count_next;

    error_header_early_termination_reg <= error_header_early_termination_next;
    error_payload_early_termination_reg <= error_payload_early_termination_next;
    error_invalid_header_reg <= error_invalid_header_next;
    error_invalid_checksum_reg <= error_invalid_checksum_next;

    busy_reg <= state_next != STATE_IDLE;

    // datapath
    if (store_eth_hdr) begin
        m_eth_dest_mac_reg <= s_eth_dest_mac;
        m_eth_src_mac_reg <= s_eth_src_mac;
        m_eth_type_reg <= s_eth_type;
    end

    if (store_hdr) begin
        m_ip_version_reg <= hdr_version;
        m_ip_ihl_reg <= hdr_ihl;
        m_ip_dscp_reg <= hdr_dscp;
        m_ip_ecn_reg <= hdr_ecn;
        m_ip_length_reg <= hdr_length;
        m_ip_identification_reg <= hdr_ident;
        m_ip_flags_reg <= hdr_flags;
        m_ip_fragment_offset_reg <= hdr_frag;
        m_ip_ttl_reg <= hdr_ttl;
        m_ip_protocol_reg <= hdr_proto;
        m_ip_header_checksum_reg <= hdr_csum;
        m_ip_source_ip_reg <= hdr_src_ip;
        m_ip_dest_ip_reg <= hdr_dst_ip;
    end

    // save register update
    // The data half ignores flush_save: a flush ends the frame, and the next frame's header beat
    // overwrites save_data before anything reads it. That keeps the header checksum (which
    // decides a flush on a one-beat bad frame) off the enable of these 576 flops.
    if (transfer_in_save) begin
        save_data_reg <= in_tdata;
        save_keep_reg <= in_tkeep;
        save_user_reg <= in_tuser;
    end
    if (flush_save) begin
        save_last_reg <= 1'b0;
        extra_cycle_reg <= 1'b0;
    end else if (transfer_in_save) begin
        save_last_reg <= in_tlast;
        // extra output beat needed if the last input beat still has payload in its high region
        extra_cycle_reg <= in_tlast && (in_tkeep[OFFSET +: LOW] != {LOW{1'b0}});
    end

    if (rst) begin
        state_reg <= STATE_IDLE;
        s_eth_hdr_ready_reg <= 1'b0;
        s_eth_payload_axis_tready_reg <= 1'b0;
        m_ip_hdr_valid_reg <= 1'b0;
        save_last_reg <= 1'b0;
        extra_cycle_reg <= 1'b0;
        busy_reg <= 1'b0;
        word_count_reg <= 16'd0;
        error_header_early_termination_reg <= 1'b0;
        error_payload_early_termination_reg <= 1'b0;
        error_invalid_header_reg <= 1'b0;
        error_invalid_checksum_reg <= 1'b0;
    end
end

// output datapath logic (skid buffer)
reg [DATA_WIDTH-1:0] m_ip_payload_axis_tdata_reg = {DATA_WIDTH{1'b0}};
reg [KEEP_WIDTH-1:0] m_ip_payload_axis_tkeep_reg = {KEEP_WIDTH{1'b0}};
reg                  m_ip_payload_axis_tvalid_reg = 1'b0, m_ip_payload_axis_tvalid_next;
reg                  m_ip_payload_axis_tlast_reg = 1'b0;
reg                  m_ip_payload_axis_tuser_reg = 1'b0;

reg [DATA_WIDTH-1:0] temp_m_ip_payload_axis_tdata_reg = {DATA_WIDTH{1'b0}};
reg [KEEP_WIDTH-1:0] temp_m_ip_payload_axis_tkeep_reg = {KEEP_WIDTH{1'b0}};
reg                  temp_m_ip_payload_axis_tvalid_reg = 1'b0, temp_m_ip_payload_axis_tvalid_next;
reg                  temp_m_ip_payload_axis_tlast_reg = 1'b0;
reg                  temp_m_ip_payload_axis_tuser_reg = 1'b0;

reg store_ip_payload_int_to_output;
reg store_ip_payload_int_to_temp;
reg store_ip_payload_axis_temp_to_output;

assign m_ip_payload_axis_tdata = m_ip_payload_axis_tdata_reg;
assign m_ip_payload_axis_tkeep = KEEP_ENABLE ? m_ip_payload_axis_tkeep_reg : {KEEP_WIDTH{1'b1}};
assign m_ip_payload_axis_tvalid = m_ip_payload_axis_tvalid_reg;
assign m_ip_payload_axis_tlast = m_ip_payload_axis_tlast_reg;
assign m_ip_payload_axis_tuser = m_ip_payload_axis_tuser_reg;

assign m_ip_payload_axis_tready_int_early = m_ip_payload_axis_tready || (!temp_m_ip_payload_axis_tvalid_reg && (!m_ip_payload_axis_tvalid_reg || !m_ip_payload_axis_tvalid_int));

always @* begin
    m_ip_payload_axis_tvalid_next = m_ip_payload_axis_tvalid_reg;
    temp_m_ip_payload_axis_tvalid_next = temp_m_ip_payload_axis_tvalid_reg;

    store_ip_payload_int_to_output = 1'b0;
    store_ip_payload_int_to_temp = 1'b0;
    store_ip_payload_axis_temp_to_output = 1'b0;

    if (m_ip_payload_axis_tready_int_reg) begin
        if (m_ip_payload_axis_tready || !m_ip_payload_axis_tvalid_reg) begin
            m_ip_payload_axis_tvalid_next = m_ip_payload_axis_tvalid_int;
            store_ip_payload_int_to_output = 1'b1;
        end else begin
            temp_m_ip_payload_axis_tvalid_next = m_ip_payload_axis_tvalid_int;
            store_ip_payload_int_to_temp = 1'b1;
        end
    end else if (m_ip_payload_axis_tready) begin
        m_ip_payload_axis_tvalid_next = temp_m_ip_payload_axis_tvalid_reg;
        temp_m_ip_payload_axis_tvalid_next = 1'b0;
        store_ip_payload_axis_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_ip_payload_axis_tvalid_reg <= 1'b0;
        m_ip_payload_axis_tready_int_reg <= 1'b0;
        temp_m_ip_payload_axis_tvalid_reg <= 1'b0;
    end else begin
        m_ip_payload_axis_tvalid_reg <= m_ip_payload_axis_tvalid_next;
        m_ip_payload_axis_tready_int_reg <= m_ip_payload_axis_tready_int_early;
        temp_m_ip_payload_axis_tvalid_reg <= temp_m_ip_payload_axis_tvalid_next;
    end

    if (store_ip_payload_int_to_output) begin
        m_ip_payload_axis_tdata_reg <= m_ip_payload_axis_tdata_int;
        m_ip_payload_axis_tkeep_reg <= m_ip_payload_axis_tkeep_int;
        m_ip_payload_axis_tlast_reg <= m_ip_payload_axis_tlast_int;
        m_ip_payload_axis_tuser_reg <= m_ip_payload_axis_tuser_int;
    end else if (store_ip_payload_axis_temp_to_output) begin
        m_ip_payload_axis_tdata_reg <= temp_m_ip_payload_axis_tdata_reg;
        m_ip_payload_axis_tkeep_reg <= temp_m_ip_payload_axis_tkeep_reg;
        m_ip_payload_axis_tlast_reg <= temp_m_ip_payload_axis_tlast_reg;
        m_ip_payload_axis_tuser_reg <= temp_m_ip_payload_axis_tuser_reg;
    end

    if (store_ip_payload_int_to_temp) begin
        temp_m_ip_payload_axis_tdata_reg <= m_ip_payload_axis_tdata_int;
        temp_m_ip_payload_axis_tkeep_reg <= m_ip_payload_axis_tkeep_int;
        temp_m_ip_payload_axis_tlast_reg <= m_ip_payload_axis_tlast_int;
        temp_m_ip_payload_axis_tuser_reg <= m_ip_payload_axis_tuser_int;
    end
end

endmodule

`resetall
