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
 * IP ethernet frame transmitter (IP frame in, Ethernet frame out, wide datapath)
 *
 * Parameterized rewrite of ip_eth_tx_64. The 20-byte IPv4 header (IHL=5) is
 * emitted in the low bytes of the first output beat, followed by the payload
 * realigned up by 20 bytes. Requires KEEP_WIDTH > 20.
 */
module ip_eth_tx_512 #
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
    input  wire [5:0]             s_ip_dscp,
    input  wire [1:0]             s_ip_ecn,
    input  wire [15:0]            s_ip_length,
    input  wire [15:0]            s_ip_identification,
    input  wire [2:0]             s_ip_flags,
    input  wire [12:0]            s_ip_fragment_offset,
    input  wire [7:0]             s_ip_ttl,
    input  wire [7:0]             s_ip_protocol,
    input  wire [31:0]            s_ip_source_ip,
    input  wire [31:0]            s_ip_dest_ip,
    input  wire [DATA_WIDTH-1:0]  s_ip_payload_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]  s_ip_payload_axis_tkeep,
    input  wire                   s_ip_payload_axis_tvalid,
    output wire                   s_ip_payload_axis_tready,
    input  wire                   s_ip_payload_axis_tlast,
    input  wire                   s_ip_payload_axis_tuser,

    /*
     * Ethernet frame output
     */
    output wire                   m_eth_hdr_valid,
    input  wire                   m_eth_hdr_ready,
    output wire [47:0]            m_eth_dest_mac,
    output wire [47:0]            m_eth_src_mac,
    output wire [15:0]            m_eth_type,
    output wire [DATA_WIDTH-1:0]  m_eth_payload_axis_tdata,
    output wire [KEEP_WIDTH-1:0]  m_eth_payload_axis_tkeep,
    output wire                   m_eth_payload_axis_tvalid,
    input  wire                   m_eth_payload_axis_tready,
    output wire                   m_eth_payload_axis_tlast,
    output wire                   m_eth_payload_axis_tuser,

    /*
     * Status signals
     */
    output wire                   busy,
    output wire                   error_payload_early_termination
);

localparam HDR_LEN = 20;
localparam OFFSET = HDR_LEN;
localparam LOW = KEEP_WIDTH - OFFSET;
localparam CL = $clog2(KEEP_WIDTH+1);

initial begin
    if (KEEP_WIDTH * 8 != DATA_WIDTH) begin
        $error("Error: AXI stream interface requires byte (8-bit) granularity (instance %m)");
        $finish;
    end
    if (KEEP_WIDTH <= HDR_LEN) begin
        $error("Error: ip_eth_tx_512 requires KEEP_WIDTH > 20 (instance %m)");
        $finish;
    end
end

localparam [1:0]
    STATE_IDLE = 2'd0,
    STATE_WRITE_FIRST = 2'd1,
    STATE_WRITE_PAYLOAD = 2'd2,
    STATE_WAIT_LAST = 2'd3;

reg [1:0] state_reg = STATE_IDLE, state_next;

reg store_ip_hdr;
reg flush_save;
reg transfer_in_save;

reg [15:0] word_count_reg = 16'd0, word_count_next;  // remaining eth-payload bytes (header+payload)

reg [5:0]  ip_dscp_reg = 6'd0;
reg [1:0]  ip_ecn_reg = 2'd0;
reg [15:0] ip_length_reg = 16'd0;
reg [15:0] ip_identification_reg = 16'd0;
reg [2:0]  ip_flags_reg = 3'd0;
reg [12:0] ip_fragment_offset_reg = 13'd0;
reg [7:0]  ip_ttl_reg = 8'd0;
reg [7:0]  ip_protocol_reg = 8'd0;
reg [31:0] ip_source_ip_reg = 32'd0;
reg [31:0] ip_dest_ip_reg = 32'd0;
reg [15:0] ip_checksum_reg = 16'd0;

reg s_ip_hdr_ready_reg = 1'b0, s_ip_hdr_ready_next;
reg s_ip_payload_axis_tready_reg = 1'b0, s_ip_payload_axis_tready_next;

reg m_eth_hdr_valid_reg = 1'b0, m_eth_hdr_valid_next;
reg [47:0] m_eth_dest_mac_reg = 48'd0;
reg [47:0] m_eth_src_mac_reg = 48'd0;
reg [15:0] m_eth_type_reg = 16'd0;

reg busy_reg = 1'b0;
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

reg [DATA_WIDTH-1:0] m_eth_payload_axis_tdata_int;
reg [KEEP_WIDTH-1:0] m_eth_payload_axis_tkeep_int;
reg                  m_eth_payload_axis_tvalid_int;
reg                  m_eth_payload_axis_tready_int_reg = 1'b0;
reg                  m_eth_payload_axis_tlast_int;
reg                  m_eth_payload_axis_tuser_int;
wire                 m_eth_payload_axis_tready_int_early;

assign s_ip_hdr_ready = s_ip_hdr_ready_reg;
assign s_ip_payload_axis_tready = s_ip_payload_axis_tready_reg;

assign m_eth_hdr_valid = m_eth_hdr_valid_reg;
assign m_eth_dest_mac = m_eth_dest_mac_reg;
assign m_eth_src_mac = m_eth_src_mac_reg;
assign m_eth_type = m_eth_type_reg;

assign busy = busy_reg;
assign error_payload_early_termination = error_payload_early_termination_reg;

// IP header checksum (combinational from inputs at handshake)
wire [19:0] csum_acc = {8'h45, s_ip_dscp, s_ip_ecn} + s_ip_length + s_ip_identification
    + {s_ip_flags, s_ip_fragment_offset} + {s_ip_ttl, s_ip_protocol}
    + s_ip_source_ip[31:16] + s_ip_source_ip[15:0] + s_ip_dest_ip[31:16] + s_ip_dest_ip[15:0];
wire [16:0] csum_f1 = csum_acc[15:0] + csum_acc[19:16];
wire [15:0] csum_f2 = csum_f1[15:0] + csum_f1[16];
wire [15:0] ip_checksum_comb = ~csum_f2;

// header bytes (low 20 bytes of first beat)
wire [HDR_LEN*8-1:0] hdr_word = {
    ip_dest_ip_reg[7:0], ip_dest_ip_reg[15:8], ip_dest_ip_reg[23:16], ip_dest_ip_reg[31:24],
    ip_source_ip_reg[7:0], ip_source_ip_reg[15:8], ip_source_ip_reg[23:16], ip_source_ip_reg[31:24],
    ip_checksum_reg[7:0], ip_checksum_reg[15:8],
    ip_protocol_reg, ip_ttl_reg,
    ip_fragment_offset_reg[7:0], {ip_flags_reg, ip_fragment_offset_reg[12:8]},
    ip_identification_reg[7:0], ip_identification_reg[15:8],
    ip_length_reg[7:0], ip_length_reg[15:8],
    {ip_dscp_reg, ip_ecn_reg},
    8'h45
};

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

// realignment shifter for payload beats (n>=1):
// output low OFFSET bytes from saved high OFFSET bytes; output high LOW bytes from current low LOW bytes
always @* begin
    shift_data[0 +: OFFSET*8] = save_data_reg[LOW*8 +: OFFSET*8];
    shift_keep[0 +: OFFSET]   = save_keep_reg[LOW +: OFFSET];

    if (extra_cycle_reg) begin
        shift_data[OFFSET*8 +: LOW*8] = {(LOW*8){1'b0}};
        shift_keep[OFFSET +: LOW]     = {LOW{1'b0}};
        shift_valid = 1'b1;
        shift_last  = save_last_reg;
        shift_user  = save_user_reg;
        shift_s_tready = flush_save;
    end else begin
        shift_data[OFFSET*8 +: LOW*8] = s_ip_payload_axis_tdata[0 +: LOW*8];
        shift_keep[OFFSET +: LOW]     = s_ip_payload_axis_tkeep[0 +: LOW];
        shift_valid = s_ip_payload_axis_tvalid;
        shift_last  = s_ip_payload_axis_tlast && (s_ip_payload_axis_tkeep[LOW +: OFFSET] == {OFFSET{1'b0}});
        shift_user  = s_ip_payload_axis_tuser && (s_ip_payload_axis_tkeep[LOW +: OFFSET] == {OFFSET{1'b0}});
        shift_s_tready = !(s_ip_payload_axis_tlast && s_ip_payload_axis_tvalid && transfer_in_save);
    end
end

always @* begin
    state_next = STATE_IDLE;

    s_ip_hdr_ready_next = 1'b0;
    s_ip_payload_axis_tready_next = 1'b0;

    store_ip_hdr = 1'b0;
    flush_save = 1'b0;
    transfer_in_save = 1'b0;

    word_count_next = word_count_reg;

    m_eth_hdr_valid_next = m_eth_hdr_valid_reg && !m_eth_hdr_ready;

    error_payload_early_termination_next = 1'b0;

    m_eth_payload_axis_tdata_int = {DATA_WIDTH{1'b0}};
    m_eth_payload_axis_tkeep_int = {KEEP_WIDTH{1'b0}};
    m_eth_payload_axis_tvalid_int = 1'b0;
    m_eth_payload_axis_tlast_int = 1'b0;
    m_eth_payload_axis_tuser_int = 1'b0;

    case (state_reg)
        STATE_IDLE: begin
            flush_save = 1'b1;
            s_ip_hdr_ready_next = !m_eth_hdr_valid_next;

            if (s_ip_hdr_ready && s_ip_hdr_valid) begin
                store_ip_hdr = 1'b1;
                s_ip_hdr_ready_next = 1'b0;
                m_eth_hdr_valid_next = 1'b1;
                word_count_next = s_ip_length;        // total eth-payload bytes
                state_next = STATE_WRITE_FIRST;
            end else begin
                state_next = STATE_IDLE;
            end
        end
        STATE_WRITE_FIRST: begin
            // emit beat0 = header (low 20) + first payload bytes (high LOW); needs first payload beat
            s_ip_payload_axis_tready_next = m_eth_payload_axis_tready_int_early;

            m_eth_payload_axis_tdata_int[0 +: OFFSET*8]   = hdr_word;
            m_eth_payload_axis_tdata_int[OFFSET*8 +: LOW*8] = s_ip_payload_axis_tdata[0 +: LOW*8];
            m_eth_payload_axis_tkeep_int = {KEEP_WIDTH{1'b1}};

            if (s_ip_payload_axis_tready && s_ip_payload_axis_tvalid) begin
                m_eth_payload_axis_tvalid_int = 1'b1;
                transfer_in_save = 1'b1;

                if (word_count_reg <= KEEP_WIDTH) begin
                    // whole frame fits in beat0
                    m_eth_payload_axis_tkeep_int = count2keep(word_count_reg[CL-1:0]);
                    m_eth_payload_axis_tlast_int = 1'b1;
                    word_count_next = 16'd0;
                    if (s_ip_payload_axis_tlast) begin
                        s_ip_payload_axis_tready_next = 1'b0;
                        flush_save = 1'b1;
                        s_ip_hdr_ready_next = !m_eth_hdr_valid_next;
                        state_next = STATE_IDLE;
                    end else begin
                        // more payload than ip_length declares; drop remainder
                        state_next = STATE_WAIT_LAST;
                    end
                end else begin
                    word_count_next = word_count_reg - KEEP_WIDTH;
                    state_next = STATE_WRITE_PAYLOAD;
                end
            end else begin
                state_next = STATE_WRITE_FIRST;
            end
        end
        STATE_WRITE_PAYLOAD: begin
            s_ip_payload_axis_tready_next = m_eth_payload_axis_tready_int_early && shift_s_tready;

            m_eth_payload_axis_tdata_int = shift_data;
            m_eth_payload_axis_tkeep_int = shift_keep;
            m_eth_payload_axis_tlast_int = shift_last;
            m_eth_payload_axis_tuser_int = shift_user;

            if (m_eth_payload_axis_tready_int_reg && shift_valid) begin
                transfer_in_save = 1'b1;
                m_eth_payload_axis_tvalid_int = 1'b1;

                if (word_count_reg <= KEEP_WIDTH) begin
                    m_eth_payload_axis_tkeep_int = shift_keep & count2keep(word_count_reg[CL-1:0]);
                    m_eth_payload_axis_tlast_int = 1'b1;
                    word_count_next = 16'd0;

                    if (shift_last) begin
                        if (keep2count(shift_keep) < word_count_reg) begin
                            error_payload_early_termination_next = 1'b1;
                            m_eth_payload_axis_tuser_int = 1'b1;
                        end
                        s_ip_payload_axis_tready_next = 1'b0;
                        flush_save = 1'b1;
                        s_ip_hdr_ready_next = !m_eth_hdr_valid_next;
                        state_next = STATE_IDLE;
                    end else begin
                        s_ip_payload_axis_tready_next = shift_s_tready;
                        state_next = STATE_WAIT_LAST;
                    end
                end else begin
                    word_count_next = word_count_reg - KEEP_WIDTH;
                    if (shift_last) begin
                        error_payload_early_termination_next = 1'b1;
                        m_eth_payload_axis_tuser_int = 1'b1;
                        s_ip_payload_axis_tready_next = 1'b0;
                        flush_save = 1'b1;
                        s_ip_hdr_ready_next = !m_eth_hdr_valid_next;
                        state_next = STATE_IDLE;
                    end else begin
                        state_next = STATE_WRITE_PAYLOAD;
                    end
                end
            end else begin
                state_next = STATE_WRITE_PAYLOAD;
            end
        end
        STATE_WAIT_LAST: begin
            s_ip_payload_axis_tready_next = shift_s_tready;

            if (shift_valid) begin
                transfer_in_save = 1'b1;
                if (shift_last) begin
                    s_ip_payload_axis_tready_next = 1'b0;
                    flush_save = 1'b1;
                    s_ip_hdr_ready_next = !m_eth_hdr_valid_next;
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

    m_eth_hdr_valid_reg <= m_eth_hdr_valid_next;

    word_count_reg <= word_count_next;

    error_payload_early_termination_reg <= error_payload_early_termination_next;

    busy_reg <= state_next != STATE_IDLE;

    if (store_ip_hdr) begin
        m_eth_dest_mac_reg <= s_eth_dest_mac;
        m_eth_src_mac_reg <= s_eth_src_mac;
        m_eth_type_reg <= s_eth_type;
        ip_dscp_reg <= s_ip_dscp;
        ip_ecn_reg <= s_ip_ecn;
        ip_length_reg <= s_ip_length;
        ip_identification_reg <= s_ip_identification;
        ip_flags_reg <= s_ip_flags;
        ip_fragment_offset_reg <= s_ip_fragment_offset;
        ip_ttl_reg <= s_ip_ttl;
        ip_protocol_reg <= s_ip_protocol;
        ip_source_ip_reg <= s_ip_source_ip;
        ip_dest_ip_reg <= s_ip_dest_ip;
        ip_checksum_reg <= ip_checksum_comb;
    end

    if (flush_save) begin
        save_last_reg <= 1'b0;
        extra_cycle_reg <= 1'b0;
    end else if (transfer_in_save) begin
        save_data_reg <= s_ip_payload_axis_tdata;
        save_keep_reg <= s_ip_payload_axis_tkeep;
        save_last_reg <= s_ip_payload_axis_tlast;
        save_user_reg <= s_ip_payload_axis_tuser;
        extra_cycle_reg <= s_ip_payload_axis_tlast && (s_ip_payload_axis_tkeep[LOW +: OFFSET] != {OFFSET{1'b0}});
    end

    if (rst) begin
        state_reg <= STATE_IDLE;
        s_ip_hdr_ready_reg <= 1'b0;
        s_ip_payload_axis_tready_reg <= 1'b0;
        m_eth_hdr_valid_reg <= 1'b0;
        save_last_reg <= 1'b0;
        extra_cycle_reg <= 1'b0;
        busy_reg <= 1'b0;
        word_count_reg <= 16'd0;
        error_payload_early_termination_reg <= 1'b0;
    end
end

// output datapath logic (skid buffer)
reg [DATA_WIDTH-1:0] m_eth_payload_axis_tdata_reg = {DATA_WIDTH{1'b0}};
reg [KEEP_WIDTH-1:0] m_eth_payload_axis_tkeep_reg = {KEEP_WIDTH{1'b0}};
reg                  m_eth_payload_axis_tvalid_reg = 1'b0, m_eth_payload_axis_tvalid_next;
reg                  m_eth_payload_axis_tlast_reg = 1'b0;
reg                  m_eth_payload_axis_tuser_reg = 1'b0;

reg [DATA_WIDTH-1:0] temp_m_eth_payload_axis_tdata_reg = {DATA_WIDTH{1'b0}};
reg [KEEP_WIDTH-1:0] temp_m_eth_payload_axis_tkeep_reg = {KEEP_WIDTH{1'b0}};
reg                  temp_m_eth_payload_axis_tvalid_reg = 1'b0, temp_m_eth_payload_axis_tvalid_next;
reg                  temp_m_eth_payload_axis_tlast_reg = 1'b0;
reg                  temp_m_eth_payload_axis_tuser_reg = 1'b0;

reg store_eth_payload_int_to_output;
reg store_eth_payload_int_to_temp;
reg store_eth_payload_axis_temp_to_output;

assign m_eth_payload_axis_tdata = m_eth_payload_axis_tdata_reg;
assign m_eth_payload_axis_tkeep = KEEP_ENABLE ? m_eth_payload_axis_tkeep_reg : {KEEP_WIDTH{1'b1}};
assign m_eth_payload_axis_tvalid = m_eth_payload_axis_tvalid_reg;
assign m_eth_payload_axis_tlast = m_eth_payload_axis_tlast_reg;
assign m_eth_payload_axis_tuser = m_eth_payload_axis_tuser_reg;

assign m_eth_payload_axis_tready_int_early = m_eth_payload_axis_tready || (!temp_m_eth_payload_axis_tvalid_reg && (!m_eth_payload_axis_tvalid_reg || !m_eth_payload_axis_tvalid_int));

always @* begin
    m_eth_payload_axis_tvalid_next = m_eth_payload_axis_tvalid_reg;
    temp_m_eth_payload_axis_tvalid_next = temp_m_eth_payload_axis_tvalid_reg;

    store_eth_payload_int_to_output = 1'b0;
    store_eth_payload_int_to_temp = 1'b0;
    store_eth_payload_axis_temp_to_output = 1'b0;

    if (m_eth_payload_axis_tready_int_reg) begin
        if (m_eth_payload_axis_tready || !m_eth_payload_axis_tvalid_reg) begin
            m_eth_payload_axis_tvalid_next = m_eth_payload_axis_tvalid_int;
            store_eth_payload_int_to_output = 1'b1;
        end else begin
            temp_m_eth_payload_axis_tvalid_next = m_eth_payload_axis_tvalid_int;
            store_eth_payload_int_to_temp = 1'b1;
        end
    end else if (m_eth_payload_axis_tready) begin
        m_eth_payload_axis_tvalid_next = temp_m_eth_payload_axis_tvalid_reg;
        temp_m_eth_payload_axis_tvalid_next = 1'b0;
        store_eth_payload_axis_temp_to_output = 1'b1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        m_eth_payload_axis_tvalid_reg <= 1'b0;
        m_eth_payload_axis_tready_int_reg <= 1'b0;
        temp_m_eth_payload_axis_tvalid_reg <= 1'b0;
    end else begin
        m_eth_payload_axis_tvalid_reg <= m_eth_payload_axis_tvalid_next;
        m_eth_payload_axis_tready_int_reg <= m_eth_payload_axis_tready_int_early;
        temp_m_eth_payload_axis_tvalid_reg <= temp_m_eth_payload_axis_tvalid_next;
    end

    if (store_eth_payload_int_to_output) begin
        m_eth_payload_axis_tdata_reg <= m_eth_payload_axis_tdata_int;
        m_eth_payload_axis_tkeep_reg <= m_eth_payload_axis_tkeep_int;
        m_eth_payload_axis_tlast_reg <= m_eth_payload_axis_tlast_int;
        m_eth_payload_axis_tuser_reg <= m_eth_payload_axis_tuser_int;
    end else if (store_eth_payload_axis_temp_to_output) begin
        m_eth_payload_axis_tdata_reg <= temp_m_eth_payload_axis_tdata_reg;
        m_eth_payload_axis_tkeep_reg <= temp_m_eth_payload_axis_tkeep_reg;
        m_eth_payload_axis_tlast_reg <= temp_m_eth_payload_axis_tlast_reg;
        m_eth_payload_axis_tuser_reg <= temp_m_eth_payload_axis_tuser_reg;
    end

    if (store_eth_payload_int_to_temp) begin
        temp_m_eth_payload_axis_tdata_reg <= m_eth_payload_axis_tdata_int;
        temp_m_eth_payload_axis_tkeep_reg <= m_eth_payload_axis_tkeep_int;
        temp_m_eth_payload_axis_tlast_reg <= m_eth_payload_axis_tlast_int;
        temp_m_eth_payload_axis_tuser_reg <= m_eth_payload_axis_tuser_int;
    end
end

endmodule

`resetall
