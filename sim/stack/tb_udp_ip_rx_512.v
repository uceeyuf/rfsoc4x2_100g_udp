`timescale 1ns / 1ps
`default_nettype none

// Self-checking testbench for udp_ip_rx_512
module tb_udp_ip_rx_512;

localparam DATA_WIDTH = 512;
localparam KEEP_WIDTH = DATA_WIDTH/8;

reg clk = 0;
reg rst = 1;
always #2.5 clk = ~clk;

// IP frame input
reg         s_ip_hdr_valid = 0;
wire        s_ip_hdr_ready;
reg  [15:0] s_ip_length = 0;
reg  [31:0] s_ip_source_ip = 32'hC0A80482;
reg  [31:0] s_ip_dest_ip = 32'hC0A80480;
reg  [DATA_WIDTH-1:0] s_ip_payload_axis_tdata = 0;
reg  [KEEP_WIDTH-1:0] s_ip_payload_axis_tkeep = 0;
reg         s_ip_payload_axis_tvalid = 0;
wire        s_ip_payload_axis_tready;
reg         s_ip_payload_axis_tlast = 0;
reg         s_ip_payload_axis_tuser = 0;

// UDP frame output
wire        m_udp_hdr_valid;
reg         m_udp_hdr_ready = 1;
wire [15:0] m_udp_source_port, m_udp_dest_port, m_udp_length, m_udp_checksum;
wire [31:0] m_ip_source_ip, m_ip_dest_ip;
wire [DATA_WIDTH-1:0] m_udp_payload_axis_tdata;
wire [KEEP_WIDTH-1:0] m_udp_payload_axis_tkeep;
wire        m_udp_payload_axis_tvalid;
reg         m_udp_payload_axis_tready = 1;
reg         backpressure = 0;
reg  [15:0] lfsr = 16'hBEEF;
always @(posedge clk) begin
    lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
    m_udp_payload_axis_tready <= backpressure ? lfsr[0] : 1'b1;
end
wire        m_udp_payload_axis_tlast;
wire        m_udp_payload_axis_tuser;

udp_ip_rx_512 #(.DATA_WIDTH(DATA_WIDTH)) dut (
    .clk(clk), .rst(rst),
    .s_ip_hdr_valid(s_ip_hdr_valid), .s_ip_hdr_ready(s_ip_hdr_ready),
    .s_eth_dest_mac(48'h0), .s_eth_src_mac(48'h0), .s_eth_type(16'h0800),
    .s_ip_version(4'd4), .s_ip_ihl(4'd5), .s_ip_dscp(6'd0), .s_ip_ecn(2'd0),
    .s_ip_length(s_ip_length), .s_ip_identification(16'h0), .s_ip_flags(3'd0),
    .s_ip_fragment_offset(13'd0), .s_ip_ttl(8'd64), .s_ip_protocol(8'd17),
    .s_ip_header_checksum(16'h0), .s_ip_source_ip(s_ip_source_ip), .s_ip_dest_ip(s_ip_dest_ip),
    .s_ip_payload_axis_tdata(s_ip_payload_axis_tdata),
    .s_ip_payload_axis_tkeep(s_ip_payload_axis_tkeep),
    .s_ip_payload_axis_tvalid(s_ip_payload_axis_tvalid),
    .s_ip_payload_axis_tready(s_ip_payload_axis_tready),
    .s_ip_payload_axis_tlast(s_ip_payload_axis_tlast),
    .s_ip_payload_axis_tuser(s_ip_payload_axis_tuser),
    .m_udp_hdr_valid(m_udp_hdr_valid), .m_udp_hdr_ready(m_udp_hdr_ready),
    .m_eth_dest_mac(), .m_eth_src_mac(), .m_eth_type(),
    .m_ip_version(), .m_ip_ihl(), .m_ip_dscp(), .m_ip_ecn(),
    .m_ip_length(), .m_ip_identification(), .m_ip_flags(), .m_ip_fragment_offset(),
    .m_ip_ttl(), .m_ip_protocol(), .m_ip_header_checksum(),
    .m_ip_source_ip(m_ip_source_ip), .m_ip_dest_ip(m_ip_dest_ip),
    .m_udp_source_port(m_udp_source_port), .m_udp_dest_port(m_udp_dest_port),
    .m_udp_length(m_udp_length), .m_udp_checksum(m_udp_checksum),
    .m_udp_payload_axis_tdata(m_udp_payload_axis_tdata),
    .m_udp_payload_axis_tkeep(m_udp_payload_axis_tkeep),
    .m_udp_payload_axis_tvalid(m_udp_payload_axis_tvalid),
    .m_udp_payload_axis_tready(m_udp_payload_axis_tready),
    .m_udp_payload_axis_tlast(m_udp_payload_axis_tlast),
    .m_udp_payload_axis_tuser(m_udp_payload_axis_tuser),
    .busy(), .error_header_early_termination(), .error_payload_early_termination()
);

reg [7:0] dbuf [0:2047];   // IP payload (UDP datagram) bytes
integer frame_len;
reg [7:0] expbuf [0:2047];
integer exp_len;
integer udp_len;
integer errors = 0;

task build_udp;
    input integer payload_len;
    input integer pad_to;
    integer i;
    begin
        udp_len = 8 + payload_len;
        dbuf[0] = 8'h12; dbuf[1] = 8'h34;          // src port 0x1234
        dbuf[2] = 8'h04; dbuf[3] = 8'hD2;          // dst port 1234
        dbuf[4] = (udp_len >> 8) & 8'hff; dbuf[5] = udp_len & 8'hff;
        dbuf[6] = 8'h00; dbuf[7] = 8'h00;          // checksum unused
        for (i = 0; i < payload_len; i = i + 1) begin
            dbuf[8+i] = (i + 8'h21) & 8'hff;
            expbuf[i] = (i + 8'h21) & 8'hff;
        end
        exp_len = payload_len;
        frame_len = udp_len;
        if (pad_to > frame_len) begin
            for (i = frame_len; i < pad_to; i = i + 1) dbuf[i] = 8'h00;
            frame_len = pad_to;
        end
    end
endtask

task send_frame;
    integer pos, j, nbytes;
    reg [DATA_WIDTH-1:0] d;
    reg [KEEP_WIDTH-1:0] k;
    begin
        @(posedge clk);
        s_ip_length <= 20 + udp_len;
        s_ip_hdr_valid <= 1;
        @(posedge clk);
        while (!s_ip_hdr_ready) @(posedge clk);
        s_ip_hdr_valid <= 0;

        pos = 0;
        while (pos < frame_len) begin
            d = 0; k = 0;
            nbytes = frame_len - pos;
            if (nbytes > KEEP_WIDTH) nbytes = KEEP_WIDTH;
            for (j = 0; j < nbytes; j = j + 1) begin
                d[j*8 +: 8] = dbuf[pos+j];
                k[j] = 1'b1;
            end
            s_ip_payload_axis_tdata <= d;
            s_ip_payload_axis_tkeep <= k;
            s_ip_payload_axis_tvalid <= 1;
            s_ip_payload_axis_tlast <= (pos + nbytes >= frame_len);
            @(posedge clk);
            while (!s_ip_payload_axis_tready) @(posedge clk);
            pos = pos + nbytes;
        end
        s_ip_payload_axis_tvalid <= 0;
        s_ip_payload_axis_tlast <= 0;
    end
endtask

reg [7:0] gotbuf [0:2047];
integer got_len;
always @(posedge clk) begin
    if (rst) got_len <= 0;
    else if (m_udp_payload_axis_tvalid && m_udp_payload_axis_tready) begin : collect
        integer b;
        for (b = 0; b < KEEP_WIDTH; b = b + 1)
            if (m_udp_payload_axis_tkeep[b]) begin
                gotbuf[got_len] = m_udp_payload_axis_tdata[b*8 +: 8];
                got_len = got_len + 1;
            end
    end
end

reg hdr_seen;
always @(posedge clk) begin
    if (m_udp_hdr_valid && m_udp_hdr_ready) begin
        hdr_seen <= 1;
        if (m_udp_source_port !== 16'h1234) begin $display("FAIL src_port %h", m_udp_source_port); errors=errors+1; end
        if (m_udp_dest_port !== 16'd1234)  begin $display("FAIL dst_port %d", m_udp_dest_port); errors=errors+1; end
        if (m_udp_length !== udp_len[15:0]) begin $display("FAIL udp_len %d exp %d", m_udp_length, udp_len); errors=errors+1; end
        if (m_ip_source_ip !== 32'hC0A80482) begin $display("FAIL src_ip %h", m_ip_source_ip); errors=errors+1; end
    end
end

task run_case;
    input [127:0] name;
    input integer plen;
    input integer pad_to;
    integer i;
    begin
        @(posedge clk);
        got_len = 0; hdr_seen = 0;
        build_udp(plen, pad_to);
        send_frame;
        repeat (40) @(posedge clk);
        if (got_len !== exp_len) begin
            $display("FAIL[%0s]: payload len %0d exp %0d", name, got_len, exp_len); errors=errors+1;
        end else begin
            for (i = 0; i < exp_len; i = i + 1)
                if (gotbuf[i] !== expbuf[i]) begin
                    $display("FAIL[%0s]: byte %0d=%h exp %h", name, i, gotbuf[i], expbuf[i]); errors=errors+1;
                end
            if (!hdr_seen) begin $display("FAIL[%0s]: no udp_hdr_valid", name); errors=errors+1; end
            else if (errors == 0) $display("PASS[%0s]: %0d payload bytes OK", name, exp_len);
        end
    end
endtask

initial begin
    repeat (8) @(posedge clk);
    rst <= 0;
    @(posedge clk);

    run_case("u1",    1,    0);
    run_case("u56",   56,   0);    // payload fills beat0 low region (64-8)
    run_case("u57",   57,   0);
    run_case("u200",  200,  0);
    run_case("u1472", 1472, 0);
    run_case("upad",  6,    60);   // short, padded

    backpressure = 1;
    run_case("bp200",  200, 0);
    run_case("bp1472", 1472,0);
    backpressure = 0;

    if (errors == 0) $display("\n*** ALL TESTS PASSED ***");
    else             $display("\n*** %0d ERRORS ***", errors);
    $finish;
end

initial begin #300000 $display("TIMEOUT"); $finish; end

endmodule

`default_nettype wire
