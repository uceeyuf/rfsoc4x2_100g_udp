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
 * UDP block, IP receive path (IP frame in, UDP frame out, wide datapath)
 *
 * Parameterized rewrite of udp_ip_rx_64. The 8-byte UDP header lands in the
 * first payload beat; the payload is realigned by 8 bytes. Requires KEEP_WIDTH > 8.
 */
module udp_ip_rx_512 #
(
    parameter DATA_WIDTH = 512,
    parameter KEEP_ENABLE = (DATA_WIDTH>8),
    parameter KEEP_WIDTH = (DATA_WIDTH/8)
)
(
    input  wire                   clk,
    input  wire                   rst,

    /*
     * IP frame input
     */
    input  wire                   s_ip_hdr_valid,
    output wire                   s_ip_hdr_ready,
    input  wire [47:0]            s_eth_dest_mac,
    input  wire [47:0]            s_eth_src_mac,
    input  wire [15:0]            s_eth_type,
    input  wire [3:0]             s_ip_version,
    input  wire [3:0]             s_ip_ihl,
    input  wire [5:0]             s_ip_dscp,
    input  wire [1:0]             s_ip_ecn,
    input  wire [15:0]            s_ip_length,
    input  wire [15:0]            s_ip_identification,
    input  wire [2:0]             s_ip_flags,
    input  wire [12:0]            s_ip_fragment_offset,
    input  wire [7:0]             s_ip_ttl,
    input  wire [7:0]             s_ip_protocol,
    input  wire [15:0]            s_ip_header_checksum,
    input  wire [31:0]            s_ip_source_ip,
    input  wire [31:0]            s_ip_dest_ip,
    input  wire [DATA_WIDTH-1:0]  s_ip_payload_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]  s_ip_payload_axis_tkeep,
    input  wire                   s_ip_payload_axis_tvalid,
    output wire                   s_ip_payload_axis_tready,
    input  wire                   s_ip_payload_axis_tlast,
    input  wire                   s_ip_payload_axis_tuser,

    /*
     * UDP frame output
     */
    output wire                   m_udp_hdr_valid,
    input  wire                   m_udp_hdr_ready,
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
    output wire [15:0]            m_udp_source_port,
    output wire [15:0]            m_udp_dest_port,
    output wire [15:0]            m_udp_length,
    output wire [15:0]            m_udp_checksum,
    output wire [DATA_WIDTH-1:0]  m_udp_payload_axis_tdata,
    output wire [KEEP_WIDTH-1:0]  m_udp_payload_axis_tkeep,
    output wire                   m_udp_payload_axis_tvalid,
    input  wire                   m_udp_payload_axis_tready,
    output wire                   m_udp_payload_axis_tlast,
    output wire                   m_udp_payload_axis_tuser,

    /*
     * Status signals
     */
    output wire                   busy,
    output wire                   error_header_early_termination,
    output wire                   error_payload_early_termination
);

localparam HDR_LEN = 8;        // UDP header
localparam OFFSET = HDR_LEN;
localparam LOW = KEEP_WIDTH - OFFSET;
localparam CL = $clog2(KEEP_WIDTH+1);

initial begin
    if (KEEP_WIDTH * 8 != DATA_WIDTH) begin
        $error("Error: AXI stream interface requires byte (8-bit) granularity (instance %m)");
        $finish;
    end
    if (KEEP_WIDTH <= HDR_LEN) begin
        $error("Error: udp_ip_rx_512 requires KEEP_WIDTH > 8 (instance %m)");
        $finish;
    end
end

localparam [1:0]
    STATE_IDLE = 2'd0,
    STATE_READ_HEADER = 2'd1,
    STATE_READ_PAYLOAD = 2'd2,
    STATE_WAIT_LAST = 2'd3;

reg [1:0] state_reg = STATE_IDLE, state_next;

reg store_ip_hdr;
reg store_hdr;
reg flush_save;
reg transfer_in_save;

reg [15:0] word_count_reg = 16'd0, word_count_next;

reg s_ip_hdr_ready_reg = 1'b0, s_ip_hdr_ready_next;
reg s_ip_payload_axis_tready_reg = 1'b0, s_ip_payload_axis_tready_next;

reg m_udp_hdr_valid_reg = 1'b0, m_udp_hdr_valid_next;
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
reg [15:0] m_udp_source_port_reg = 16'd0;
reg [15:0] m_udp_dest_port_reg = 16'd0;
reg [15:0] m_udp_length_reg = 16'd0;
reg [15:0] m_udp_checksum_reg = 16'd0;

reg busy_reg = 1'b0;
reg error_header_early_termination_reg = 1'b0, error_header_early_termination_next;
reg error_payload_early_termination_reg = 1'b0, error_payload_early_termination_next;

reg [DATA_WIDTH-1:0] save_data_reg = {DATA_WIDTH{1'b0}};
reg [KEEP_WIDTH-1:0] save_keep_reg = {KEEP_WIDTH{1'b0}};
reg save_last_reg = 1'b0;
reg save_user_reg = 1'b0;
reg extra_cycle_reg = 1'b0;

reg [DATA_WIDTH-1:0] shift_data;
reg [KEEP_WIDTH-1:0] shift_keep;
reg shift_valid;
reg shift_last;
reg shift_user;
reg shift_s_tready;

reg [DATA_WIDTH-1:0] m_udp_payload_axis_tdata_int;
reg [KEEP_WIDTH-1:0] m_udp_payload_axis_tkeep_int;
reg                  m_udp_payload_axis_tvalid_int;
reg                  m_udp_payload_axis_tready_int_reg = 1'b0;
reg                  m_udp_payload_axis_tlast_int;
reg                  m_udp_payload_axis_tuser_int;
wire                 m_udp_payload_axis_tready_int_early;

assign s_ip_hdr_ready = s_ip_hdr_ready_reg;
assign s_ip_payload_axis_tready = s_ip_payload_axis_tready_reg;

assign m_udp_hdr_valid = m_udp_hdr_valid_reg;
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
assign m_udp_source_port = m_udp_source_port_reg;
assign m_udp_dest_port = m_udp_dest_port_reg;
assign m_udp_length = m_udp_length_reg;
assign m_udp_checksum = m_udp_checksum_reg;

assign busy = busy_reg;
assign error_header_early_termination = error_header_early_termination_reg;
assign error_payload_early_termination = error_payload_early_termination_reg;

function [7:0] hbyte;
    input [DATA_WIDTH-1:0] d;
    input integer i;
    hbyte = d[i*8 +: 8];
endfunction

// UDP header fields from first payload beat
wire [15:0] hdr_src_port = {hbyte(s_ip_payload_axis_tdata, 0), hbyte(s_ip_payload_axis_tdata, 1)};
wire [15:0] hdr_dst_port = {hbyte(s_ip_payload_axis_tdata, 2), hbyte(s_ip_payload_axis_tdata, 3)};
wire [15:0] hdr_length   = {hbyte(s_ip_payload_axis_tdata, 4), hbyte(s_ip_payload_axis_tdata, 5)};
wire [15:0] hdr_checksum = {hbyte(s_ip_payload_axis_tdata, 6), hbyte(s_ip_payload_axis_tdata, 7)};

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
        shift_data[LOW*8 +: OFFSET*8] = s_ip_payload_axis_tdata[0 +: OFFSET*8];
        shift_keep[LOW +: OFFSET]     = s_ip_payload_axis_tkeep[0 +: OFFSET];
        shift_valid = s_ip_payload_axis_tvalid;
        shift_last  = s_ip_payload_axis_tlast && (s_ip_payload_axis_tkeep[OFFSET +: LOW] == {LOW{1'b0}});
        shift_user  = s_ip_payload_axis_tuser && (s_ip_payload_axis_tkeep[OFFSET +: LOW] == {LOW{1'b0}});
        shift_s_tready = !(s_ip_payload_axis_tlast && s_ip_payload_axis_tvalid && transfer_in_save);
    end
end

always @* begin
    state_next = STATE_IDLE;

    flush_save = 1'b0;
    transfer_in_save = 1'b0;

    s_ip_hdr_ready_next = 1'b0;
    s_ip_payload_axis_tready_next = 1'b0;

    store_ip_hdr = 1'b0;
    store_hdr = 1'b0;

    word_count_next = word_count_reg;

    m_udp_hdr_valid_next = m_udp_hdr_valid_reg && !m_udp_hdr_ready;

    error_header_early_termination_next = 1'b0;
    error_payload_early_termination_next = 1'b0;

    m_udp_payload_axis_tdata_int = {DATA_WIDTH{1'b0}};
    m_udp_payload_axis_tkeep_int = {KEEP_WIDTH{1'b0}};
    m_udp_payload_axis_tvalid_int = 1'b0;
    m_udp_payload_axis_tlast_int = 1'b0;
    m_udp_payload_axis_tuser_int = 1'b0;

    case (state_reg)
        STATE_IDLE: begin
            flush_save = 1'b1;
            s_ip_hdr_ready_next = !m_udp_hdr_valid_next;

            if (s_ip_hdr_ready && s_ip_hdr_valid) begin
                s_ip_hdr_ready_next = 1'b0;
                s_ip_payload_axis_tready_next = 1'b1;
                store_ip_hdr = 1'b1;
                state_next = STATE_READ_HEADER;
            end else begin
                state_next = STATE_IDLE;
            end
        end
        STATE_READ_HEADER: begin
            s_ip_payload_axis_tready_next = shift_s_tready;

            if (s_ip_payload_axis_tvalid) begin
                store_hdr = 1'b1;
                transfer_in_save = 1'b1;
                word_count_next = hdr_length - HDR_LEN;

                m_udp_hdr_valid_next = 1'b1;
                s_ip_payload_axis_tready_next = m_udp_payload_axis_tready_int_early && shift_s_tready;
                state_next = STATE_READ_PAYLOAD;

                if (s_ip_payload_axis_tlast) begin
                    // entire frame in one beat: payload (if any) flushed via extra cycle in READ_PAYLOAD
                    if (hdr_length < HDR_LEN) begin
                        error_header_early_termination_next = 1'b1;
                        m_udp_hdr_valid_next = 1'b0;
                        s_ip_payload_axis_tready_next = 1'b0;
                        flush_save = 1'b1;
                        s_ip_hdr_ready_next = 1'b1;
                        state_next = STATE_IDLE;
                    end
                end
            end else begin
                state_next = STATE_READ_HEADER;
            end
        end
        STATE_READ_PAYLOAD: begin
            s_ip_payload_axis_tready_next = m_udp_payload_axis_tready_int_early && shift_s_tready;

            m_udp_payload_axis_tdata_int = shift_data;
            m_udp_payload_axis_tkeep_int = shift_keep;
            m_udp_payload_axis_tlast_int = shift_last;
            m_udp_payload_axis_tuser_int = shift_user;

            if (m_udp_payload_axis_tready_int_reg && shift_valid) begin
                transfer_in_save = 1'b1;
                m_udp_payload_axis_tvalid_int = 1'b1;

                if (word_count_reg <= KEEP_WIDTH) begin
                    m_udp_payload_axis_tkeep_int = shift_keep & count2keep(word_count_reg[CL-1:0]);
                    m_udp_payload_axis_tlast_int = 1'b1;
                    word_count_next = 16'd0;

                    if (shift_last) begin
                        if (keep2count(shift_keep) < word_count_reg) begin
                            error_payload_early_termination_next = 1'b1;
                            m_udp_payload_axis_tuser_int = 1'b1;
                        end
                        s_ip_payload_axis_tready_next = 1'b0;
                        flush_save = 1'b1;
                        s_ip_hdr_ready_next = !m_udp_hdr_valid_next;
                        state_next = STATE_IDLE;
                    end else begin
                        s_ip_payload_axis_tready_next = shift_s_tready;
                        state_next = STATE_WAIT_LAST;
                    end
                end else begin
                    word_count_next = word_count_reg - KEEP_WIDTH;
                    if (shift_last) begin
                        error_payload_early_termination_next = 1'b1;
                        m_udp_payload_axis_tuser_int = 1'b1;
                        s_ip_payload_axis_tready_next = 1'b0;
                        flush_save = 1'b1;
                        s_ip_hdr_ready_next = !m_udp_hdr_valid_next;
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
            s_ip_payload_axis_tready_next = shift_s_tready;

            if (shift_valid) begin
                transfer_in_save = 1'b1;
                if (shift_last) begin
                    s_ip_payload_axis_tready_next = 1'b0;
                    flush_save = 1'b1;
                    s_ip_hdr_ready_next = !m_udp_hdr_valid_next;
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

    s_ip_hdr_ready_reg <= s_ip_hdr_ready_next;
    s_ip_payload_axis_tready_reg <= s_ip_payload_axis_tready_next;

    m_udp_hdr_valid_reg <= m_udp_hdr_valid_next;

    word_count_reg <= word_count_next;

    error_header_early_termination_reg <= error_header_early_termination_next;
    error_payload_early_termination_reg <= error_payload_early_termination_next;

    busy_reg <= state_next != STATE_IDLE;

    if (store_ip_hdr) begin
        m_eth_dest_mac_reg <= s_eth_dest_mac;
        m_eth_src_mac_reg <= s_eth_src_mac;
        m_eth_type_reg <= s_eth_type;
        m_ip_version_reg <= s_ip_version;
        m_ip_ihl_reg <= s_ip_ihl;
        m_ip_dscp_reg <= s_ip_dscp;
        m_ip_ecn_reg <= s_ip_ecn;
        m_ip_length_reg <= s_ip_length;
        m_ip_identification_reg <= s_ip_identification;
        m_ip_flags_reg <= s_ip_flags;
        m_ip_fragment_offset_reg <= s_ip_fragment_offset;
        m_ip_ttl_reg <= s_ip_ttl;
        m_ip_protocol_reg <= s_ip_protocol;
        m_ip_header_checksum_reg <= s_ip_header_checksum;
        m_ip_source_ip_reg <= s_ip_source_ip;
        m_ip_dest_ip_reg <= s_ip_dest_ip;
    end

    if (store_hdr) begin
        m_udp_source_port_reg <= hdr_src_port;
        m_udp_dest_port_reg <= hdr_dst_port;
        m_udp_length_reg <= hdr_length;
        m_udp_checksum_reg <= hdr_checksum;
    end

    if (flush_save) begin
        save_last_reg <= 1'b0;
        extra_cycle_reg <= 1'b0;
    end else if (transfer_in_save) begin
        save_data_reg <= s_ip_payload_axis_tdata;
        save_keep_reg <= s_ip_payload_axis_tkeep;
        save_last_reg <= s_ip_payload_axis_tlast;
        save_user_reg <= s_ip_payload_axis_tuser;
        extra_cycle_reg <= s_ip_payload_axis_tlast && (s_ip_payload_axis_tkeep[OFFSET +: LOW] != {LOW{1'b0}});
    end

    if (rst) begin
        state_reg <= STATE_IDLE;
        s_ip_hdr_ready_reg <= 1'b0;
        s_ip_payload_axis_tready_reg <= 1'b0;
        m_udp_hdr_valid_reg <= 1'b0;
        save_last_reg <= 1'b0;
        extra_cycle_reg <= 1'b0;
        busy_reg <= 1'b0;
        word_count_reg <= 16'd0;
        error_header_early_termination_reg <= 1'b0;
        error_payload_early_termination_reg <= 1'b0;
    end
end

// output datapath logic (skid buffer)
reg [DATA_WIDTH-1:0] m_udp_payload_axis_tdata_reg = {DATA_WIDTH{1'b0}};
reg [KEEP_WIDTH-1:0] m_udp_payload_axis_tkeep_reg = {KEEP_WIDTH{1'b0}};
reg                  m_udp_payload_axis_tvalid_reg = 1'b0, m_udp_payload_axis_tvalid_next;
reg                  m_udp_payload_axis_tlast_reg = 1'b0;
reg                  m_udp_payload_axis_tuser_reg = 1'b0;

reg [DATA_WIDTH-1:0] temp_m_udp_payload_axis_tdata_reg = {DATA_WIDTH{1'b0}};
reg [KEEP_WIDTH-1:0] temp_m_udp_payload_axis_tkeep_reg = {KEEP_WIDTH{1'b0}};
reg                  temp_m_udp_payload_axis_tvalid_reg = 1'b0, temp_m_udp_payload_axis_tvalid_next;
reg                  temp_m_udp_payload_axis_tlast_reg = 1'b0;
reg                  temp_m_udp_payload_axis_tuser_reg = 1'b0;

reg store_udp_payload_int_to_output;
reg store_udp_payload_int_to_temp;
reg store_udp_payload_axis_temp_to_output;

assign m_udp_payload_axis_tdata = m_udp_payload_axis_tdata_reg;
assign m_udp_payload_axis_tkeep = KEEP_ENABLE ? m_udp_payload_axis_tkeep_reg : {KEEP_WIDTH{1'b1}};
assign m_udp_payload_axis_tvalid = m_udp_payload_axis_tvalid_reg;
assign m_udp_payload_axis_tlast = m_udp_payload_axis_tlast_reg;
assign m_udp_payload_axis_tuser = m_udp_payload_axis_tuser_reg;

assign m_udp_payload_axis_tready_int_early = m_udp_payload_axis_tready || (!temp_m_udp_payload_axis_tvalid_reg && (!m_udp_payload_axis_tvalid_reg || !m_udp_payload_axis_tvalid_int));

always @* begin
    m_udp_payload_axis_tvalid_next = m_udp_payload_axis_tvalid_reg;
    temp_m_udp_payload_axis_tvalid_next = temp_m_udp_payload_axis_tvalid_reg;

    store_udp_payload_int_to_output = 1'b0;
    store_udp_payload_int_to_temp = 1'b0;
    store_udp_payload_axis_temp_to_output = 1'b0;

    if (m_udp_payload_axis_tready_int_reg) begin
        if (m_udp_payload_axis_tready || !m_udp_payload_axis_tvalid_reg) begin
            m_udp_payload_axis_tvalid_next = m_udp_payload_axis_tvalid_int;
            store_udp_payload_int_to_output = 1'b1;
        end else begin
            temp_m_udp_payload_axis_tvalid_next = m_udp_payload_axis_tvalid_int;
            store_udp_payload_int_to_temp = 1'b1;
        end
    end else if (m_udp_payload_axis_tready) begin
        m_udp_payload_axis_tvalid_next = temp_m_udp_payload_axis_tvalid_reg;
        temp_m_udp_payload_axis_tvalid_next = 1'b0;
        store_udp_payload_axis_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_udp_payload_axis_tvalid_reg <= 1'b0;
        m_udp_payload_axis_tready_int_reg <= 1'b0;
        temp_m_udp_payload_axis_tvalid_reg <= 1'b0;
    end else begin
        m_udp_payload_axis_tvalid_reg <= m_udp_payload_axis_tvalid_next;
        m_udp_payload_axis_tready_int_reg <= m_udp_payload_axis_tready_int_early;
        temp_m_udp_payload_axis_tvalid_reg <= temp_m_udp_payload_axis_tvalid_next;
    end

    if (store_udp_payload_int_to_output) begin
        m_udp_payload_axis_tdata_reg <= m_udp_payload_axis_tdata_int;
        m_udp_payload_axis_tkeep_reg <= m_udp_payload_axis_tkeep_int;
        m_udp_payload_axis_tlast_reg <= m_udp_payload_axis_tlast_int;
        m_udp_payload_axis_tuser_reg <= m_udp_payload_axis_tuser_int;
    end else if (store_udp_payload_axis_temp_to_output) begin
        m_udp_payload_axis_tdata_reg <= temp_m_udp_payload_axis_tdata_reg;
        m_udp_payload_axis_tkeep_reg <= temp_m_udp_payload_axis_tkeep_reg;
        m_udp_payload_axis_tlast_reg <= temp_m_udp_payload_axis_tlast_reg;
        m_udp_payload_axis_tuser_reg <= temp_m_udp_payload_axis_tuser_reg;
    end

    if (store_udp_payload_int_to_temp) begin
        temp_m_udp_payload_axis_tdata_reg <= m_udp_payload_axis_tdata_int;
        temp_m_udp_payload_axis_tkeep_reg <= m_udp_payload_axis_tkeep_int;
        temp_m_udp_payload_axis_tlast_reg <= m_udp_payload_axis_tlast_int;
        temp_m_udp_payload_axis_tuser_reg <= m_udp_payload_axis_tuser_int;
    end
end

endmodule

`resetall
