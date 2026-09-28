`timescale 1ns / 1ps
`default_nettype none

// Self-checking testbench for ip_eth_rx_512
module tb_ip_eth_rx_512;

localparam DATA_WIDTH = 512;
localparam KEEP_WIDTH = DATA_WIDTH/8;

reg clk = 0;
reg rst = 1;
always #2.5 clk = ~clk;   // 200 MHz

// DUT eth input
reg         s_eth_hdr_valid = 0;
wire        s_eth_hdr_ready;
reg  [47:0] s_eth_dest_mac = 48'h020000000000;
reg  [47:0] s_eth_src_mac  = 48'hDAD1D2D3D4D5;
reg  [15:0] s_eth_type     = 16'h0800;
reg  [DATA_WIDTH-1:0] s_eth_payload_axis_tdata = 0;
reg  [KEEP_WIDTH-1:0] s_eth_payload_axis_tkeep = 0;
reg         s_eth_payload_axis_tvalid = 0;
wire        s_eth_payload_axis_tready;
reg         s_eth_payload_axis_tlast = 0;
reg         s_eth_payload_axis_tuser = 0;

// DUT ip output
wire        m_ip_hdr_valid;
reg         m_ip_hdr_ready = 1;
wire [15:0] m_ip_length;
wire [7:0]  m_ip_protocol;
wire [31:0] m_ip_source_ip;
wire [31:0] m_ip_dest_ip;
wire [15:0] m_ip_header_checksum;
wire [DATA_WIDTH-1:0] m_ip_payload_axis_tdata;
wire [KEEP_WIDTH-1:0] m_ip_payload_axis_tkeep;
wire        m_ip_payload_axis_tvalid;
reg         m_ip_payload_axis_tready = 1;
reg         backpressure = 0;
reg  [15:0] lfsr = 16'hACE1;
always @(posedge clk) begin
    lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
    if (backpressure) m_ip_payload_axis_tready <= lfsr[0];
    else m_ip_payload_axis_tready <= 1'b1;
end
wire        m_ip_payload_axis_tlast;
wire        m_ip_payload_axis_tuser;
wire        error_invalid_header;
wire        error_invalid_checksum;
wire        error_payload_early_termination;

ip_eth_rx_512 #(.DATA_WIDTH(DATA_WIDTH)) dut (
    .clk(clk), .rst(rst),
    .s_eth_hdr_valid(s_eth_hdr_valid), .s_eth_hdr_ready(s_eth_hdr_ready),
    .s_eth_dest_mac(s_eth_dest_mac), .s_eth_src_mac(s_eth_src_mac), .s_eth_type(s_eth_type),
    .s_eth_payload_axis_tdata(s_eth_payload_axis_tdata),
    .s_eth_payload_axis_tkeep(s_eth_payload_axis_tkeep),
    .s_eth_payload_axis_tvalid(s_eth_payload_axis_tvalid),
    .s_eth_payload_axis_tready(s_eth_payload_axis_tready),
    .s_eth_payload_axis_tlast(s_eth_payload_axis_tlast),
    .s_eth_payload_axis_tuser(s_eth_payload_axis_tuser),
    .m_ip_hdr_valid(m_ip_hdr_valid), .m_ip_hdr_ready(m_ip_hdr_ready),
    .m_eth_dest_mac(), .m_eth_src_mac(), .m_eth_type(),
    .m_ip_version(), .m_ip_ihl(), .m_ip_dscp(), .m_ip_ecn(),
    .m_ip_length(m_ip_length), .m_ip_identification(), .m_ip_flags(), .m_ip_fragment_offset(),
    .m_ip_ttl(), .m_ip_protocol(m_ip_protocol), .m_ip_header_checksum(m_ip_header_checksum),
    .m_ip_source_ip(m_ip_source_ip), .m_ip_dest_ip(m_ip_dest_ip),
    .m_ip_payload_axis_tdata(m_ip_payload_axis_tdata),
    .m_ip_payload_axis_tkeep(m_ip_payload_axis_tkeep),
    .m_ip_payload_axis_tvalid(m_ip_payload_axis_tvalid),
    .m_ip_payload_axis_tready(m_ip_payload_axis_tready),
    .m_ip_payload_axis_tlast(m_ip_payload_axis_tlast),
    .m_ip_payload_axis_tuser(m_ip_payload_axis_tuser),
    .busy(), .error_header_early_termination(),
    .error_payload_early_termination(error_payload_early_termination),
    .error_invalid_header(error_invalid_header),
    .error_invalid_checksum(error_invalid_checksum)
);

// ---- frame builder ----
reg [7:0] ipbuf [0:2047];   // IP packet bytes (eth payload)
integer ip_len;             // IP total length
integer frame_len;          // bytes actually streamed (>= ip_len for padding test)

reg [7:0] expbuf [0:2047];  // expected IP payload bytes
integer exp_len;

integer errors = 0;

task build_ip;
    input integer payload_len;
    input integer pad_to;       // pad streamed frame to this many bytes (0 = none)
    integer i;
    integer sum;
    reg [15:0] csum;
    begin
        ip_len = 20 + payload_len;
        // header
        ipbuf[0]  = 8'h45;                 // version/ihl
        ipbuf[1]  = 8'h00;                 // dscp/ecn
        ipbuf[2]  = (ip_len >> 8) & 8'hff; // total length
        ipbuf[3]  = ip_len & 8'hff;
        ipbuf[4]  = 8'hAB;                 // ident
        ipbuf[5]  = 8'hCD;
        ipbuf[6]  = 8'h40;                 // flags=DF, frag hi
        ipbuf[7]  = 8'h00;                 // frag lo
        ipbuf[8]  = 8'd64;                 // ttl
        ipbuf[9]  = 8'd17;                 // protocol = UDP
        ipbuf[10] = 8'h00;                 // checksum (computed below)
        ipbuf[11] = 8'h00;
        ipbuf[12] = 8'd192; ipbuf[13] = 8'd168; ipbuf[14] = 8'd4; ipbuf[15] = 8'd130; // src
        ipbuf[16] = 8'd192; ipbuf[17] = 8'd168; ipbuf[18] = 8'd4; ipbuf[19] = 8'd128; // dst
        // payload
        for (i = 0; i < payload_len; i = i + 1) begin
            ipbuf[20+i] = (i + 8'h11) & 8'hff;
            expbuf[i]   = (i + 8'h11) & 8'hff;
        end
        exp_len = payload_len;
        // header checksum
        sum = 0;
        for (i = 0; i < 10; i = i + 1)
            sum = sum + ((ipbuf[2*i] << 8) | ipbuf[2*i+1]);
        sum = (sum & 16'hffff) + (sum >> 16);
        sum = (sum & 16'hffff) + (sum >> 16);
        csum = ~sum[15:0];
        ipbuf[10] = csum[15:8];
        ipbuf[11] = csum[7:0];
        // streamed length (with optional padding)
        frame_len = ip_len;
        if (pad_to > frame_len) begin
            for (i = frame_len; i < pad_to; i = i + 1) ipbuf[i] = 8'h00;
            frame_len = pad_to;
        end
    end
endtask

// stream the built frame
task send_frame;
    integer pos;
    integer j;
    integer nbytes;
    reg [DATA_WIDTH-1:0] d;
    reg [KEEP_WIDTH-1:0] k;
    begin
        // header handshake
        @(posedge clk);
        s_eth_hdr_valid <= 1;
        @(posedge clk);
        while (!s_eth_hdr_ready) @(posedge clk);
        s_eth_hdr_valid <= 0;

        // payload beats
        pos = 0;
        while (pos < frame_len) begin
            d = 0; k = 0;
            nbytes = frame_len - pos;
            if (nbytes > KEEP_WIDTH) nbytes = KEEP_WIDTH;
            for (j = 0; j < nbytes; j = j + 1) begin
                d[j*8 +: 8] = ipbuf[pos+j];
                k[j] = 1'b1;
            end
            s_eth_payload_axis_tdata <= d;
            s_eth_payload_axis_tkeep <= k;
            s_eth_payload_axis_tvalid <= 1;
            s_eth_payload_axis_tlast <= (pos + nbytes >= frame_len);
            s_eth_payload_axis_tuser <= 0;
            @(posedge clk);
            while (!s_eth_payload_axis_tready) @(posedge clk);
            pos = pos + nbytes;
        end
        s_eth_payload_axis_tvalid <= 0;
        s_eth_payload_axis_tlast <= 0;
    end
endtask

// collect output payload
reg [7:0] gotbuf [0:2047];
integer got_len;
reg collecting;

always @(posedge clk) begin
    if (rst) begin
        got_len <= 0;
    end else if (m_ip_payload_axis_tvalid && m_ip_payload_axis_tready) begin : collect
        integer b;
        for (b = 0; b < KEEP_WIDTH; b = b + 1) begin
            if (m_ip_payload_axis_tkeep[b]) begin
                gotbuf[got_len] = m_ip_payload_axis_tdata[b*8 +: 8];
                got_len = got_len + 1;
            end
        end
    end
end

// header field check
reg hdr_seen;
always @(posedge clk) begin
    if (m_ip_hdr_valid && m_ip_hdr_ready) begin
        hdr_seen <= 1;
        if (m_ip_length !== ip_len[15:0]) begin
            $display("FAIL: ip_length %0d expected %0d", m_ip_length, ip_len); errors = errors + 1;
        end
        if (m_ip_protocol !== 8'd17) begin
            $display("FAIL: protocol %0d expected 17", m_ip_protocol); errors = errors + 1;
        end
        if (m_ip_source_ip !== 32'hC0A80482) begin
            $display("FAIL: src_ip %h", m_ip_source_ip); errors = errors + 1;
        end
        if (m_ip_dest_ip !== 32'hC0A80480) begin
            $display("FAIL: dst_ip %h", m_ip_dest_ip); errors = errors + 1;
        end
    end
end

task check_payload;
    input [127:0] name;
    integer i;
    begin
        if (got_len !== exp_len) begin
            $display("FAIL[%0s]: payload len %0d expected %0d", name, got_len, exp_len);
            errors = errors + 1;
        end else begin
            for (i = 0; i < exp_len; i = i + 1)
                if (gotbuf[i] !== expbuf[i]) begin
                    $display("FAIL[%0s]: byte %0d = %h expected %h", name, i, gotbuf[i], expbuf[i]);
                    errors = errors + 1;
                end
            if (errors == 0)
                $display("PASS[%0s]: %0d payload bytes OK", name, exp_len);
        end
    end
endtask

task run_case;
    input [127:0] name;
    input integer plen;
    input integer pad_to;
    begin
        @(posedge clk);
        got_len = 0; hdr_seen = 0;
        build_ip(plen, pad_to);
        send_frame;
        // wait for output to drain
        repeat (40) @(posedge clk);
        check_payload(name);
        if (!hdr_seen) begin $display("FAIL[%0s]: no ip_hdr_valid", name); errors = errors + 1; end
    end
endtask

initial begin
    repeat (8) @(posedge clk);
    rst <= 0;
    @(posedge clk);

    run_case("small4",   4,    0);     // fits in beat0, payload 4B (->extra cycle)
    run_case("p44",      44,   0);     // payload exactly fills beat0 low region
    run_case("p45",      45,   0);     // payload spills into beat1
    run_case("p100",     100,  0);     // multi-beat
    run_case("p1472",    1472, 0);     // full MTU
    run_case("pad",      10,   60);    // short IP padded to 60B eth frame (truncate test)

    backpressure = 1;                  // downstream stalls randomly
    run_case("bp100",    100,  0);
    run_case("bp1472",   1472, 0);
    run_case("bp7",      7,    0);
    backpressure = 0;

    if (errors == 0) $display("\n*** ALL TESTS PASSED ***");
    else             $display("\n*** %0d ERRORS ***", errors);
    $finish;
end

initial begin
    #200000 $display("TIMEOUT"); $finish;
end

endmodule

`default_nettype wire
