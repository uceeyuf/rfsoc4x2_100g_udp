`timescale 1ns / 1ps
`default_nettype none

// End-to-end loopback: udp_ip_tx_512 -> ip_eth_tx_512 -> ip_eth_rx_512 -> udp_ip_rx_512
module tb_loopback_512;

localparam DW = 512;
localparam KW = DW/8;

reg clk = 0, rst = 1;
always #2.5 clk = ~clk;

// ---- stimulus into udp_ip_tx ----
reg         a_udp_hdr_valid = 0;
wire        a_udp_hdr_ready;
reg  [15:0] a_udp_source_port = 16'h1234;
reg  [15:0] a_udp_dest_port   = 16'd1234;
reg  [15:0] a_udp_length = 0;
reg  [DW-1:0] a_pl_tdata = 0;
reg  [KW-1:0] a_pl_tkeep = 0;
reg         a_pl_tvalid = 0;
wire        a_pl_tready;
reg         a_pl_tlast = 0;

// udp_ip_tx -> ip_eth_tx (IP frame)
wire        b_ip_hdr_valid; wire b_ip_hdr_ready;
wire [47:0] b_eth_dest_mac, b_eth_src_mac; wire [15:0] b_eth_type;
wire [5:0]  b_ip_dscp; wire [1:0] b_ip_ecn; wire [15:0] b_ip_length, b_ip_identification;
wire [2:0]  b_ip_flags; wire [12:0] b_ip_fragment_offset; wire [7:0] b_ip_ttl, b_ip_protocol;
wire [15:0] b_ip_header_checksum; wire [31:0] b_ip_source_ip, b_ip_dest_ip;
wire [DW-1:0] b_ip_pl_tdata; wire [KW-1:0] b_ip_pl_tkeep;
wire        b_ip_pl_tvalid, b_ip_pl_tready, b_ip_pl_tlast, b_ip_pl_tuser;

// ip_eth_tx -> ip_eth_rx (Ethernet frame)
wire        c_eth_hdr_valid, c_eth_hdr_ready;
wire [47:0] c_eth_dest_mac, c_eth_src_mac; wire [15:0] c_eth_type;
wire [DW-1:0] c_eth_pl_tdata; wire [KW-1:0] c_eth_pl_tkeep;
wire        c_eth_pl_tvalid, c_eth_pl_tready, c_eth_pl_tlast, c_eth_pl_tuser;

// ip_eth_rx -> udp_ip_rx (IP frame)
wire        d_ip_hdr_valid, d_ip_hdr_ready;
wire [47:0] d_eth_dest_mac, d_eth_src_mac; wire [15:0] d_eth_type;
wire [3:0]  d_ip_version, d_ip_ihl; wire [5:0] d_ip_dscp; wire [1:0] d_ip_ecn;
wire [15:0] d_ip_length, d_ip_identification; wire [2:0] d_ip_flags; wire [12:0] d_ip_fragment_offset;
wire [7:0]  d_ip_ttl, d_ip_protocol; wire [15:0] d_ip_header_checksum;
wire [31:0] d_ip_source_ip, d_ip_dest_ip;
wire [DW-1:0] d_ip_pl_tdata; wire [KW-1:0] d_ip_pl_tkeep;
wire        d_ip_pl_tvalid, d_ip_pl_tready, d_ip_pl_tlast, d_ip_pl_tuser;

// udp_ip_rx outputs
wire        e_udp_hdr_valid; reg e_udp_hdr_ready = 1;
wire [15:0] e_udp_source_port, e_udp_dest_port, e_udp_length, e_udp_checksum;
wire [31:0] e_ip_source_ip, e_ip_dest_ip;
wire [DW-1:0] e_udp_pl_tdata; wire [KW-1:0] e_udp_pl_tkeep;
wire        e_udp_pl_tvalid; reg e_udp_pl_tready = 1;
wire        e_udp_pl_tlast, e_udp_pl_tuser;

reg backpressure = 0;
reg [15:0] lfsr = 16'hF00D;
always @(posedge clk) begin
    lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
    e_udp_pl_tready <= backpressure ? lfsr[0] : 1'b1;
end

udp_ip_tx_512 #(.DATA_WIDTH(DW)) u_tx (
    .clk(clk), .rst(rst),
    .s_udp_hdr_valid(a_udp_hdr_valid), .s_udp_hdr_ready(a_udp_hdr_ready),
    .s_eth_dest_mac(48'h020000000011), .s_eth_src_mac(48'h020000000022), .s_eth_type(16'h0800),
    .s_ip_version(4'd4), .s_ip_ihl(4'd5), .s_ip_dscp(6'd0), .s_ip_ecn(2'd0),
    .s_ip_identification(16'hABCD), .s_ip_flags(3'b010), .s_ip_fragment_offset(13'd0),
    .s_ip_ttl(8'd64), .s_ip_protocol(8'd17), .s_ip_header_checksum(16'd0),
    .s_ip_source_ip(32'hC0A80482), .s_ip_dest_ip(32'hC0A80480),
    .s_udp_source_port(a_udp_source_port), .s_udp_dest_port(a_udp_dest_port),
    .s_udp_length(a_udp_length), .s_udp_checksum(16'd0),
    .s_udp_payload_axis_tdata(a_pl_tdata), .s_udp_payload_axis_tkeep(a_pl_tkeep),
    .s_udp_payload_axis_tvalid(a_pl_tvalid), .s_udp_payload_axis_tready(a_pl_tready),
    .s_udp_payload_axis_tlast(a_pl_tlast), .s_udp_payload_axis_tuser(1'b0),
    .m_ip_hdr_valid(b_ip_hdr_valid), .m_ip_hdr_ready(b_ip_hdr_ready),
    .m_eth_dest_mac(b_eth_dest_mac), .m_eth_src_mac(b_eth_src_mac), .m_eth_type(b_eth_type),
    .m_ip_version(), .m_ip_ihl(), .m_ip_dscp(b_ip_dscp), .m_ip_ecn(b_ip_ecn),
    .m_ip_length(b_ip_length), .m_ip_identification(b_ip_identification), .m_ip_flags(b_ip_flags),
    .m_ip_fragment_offset(b_ip_fragment_offset), .m_ip_ttl(b_ip_ttl), .m_ip_protocol(b_ip_protocol),
    .m_ip_header_checksum(b_ip_header_checksum), .m_ip_source_ip(b_ip_source_ip), .m_ip_dest_ip(b_ip_dest_ip),
    .m_ip_payload_axis_tdata(b_ip_pl_tdata), .m_ip_payload_axis_tkeep(b_ip_pl_tkeep),
    .m_ip_payload_axis_tvalid(b_ip_pl_tvalid), .m_ip_payload_axis_tready(b_ip_pl_tready),
    .m_ip_payload_axis_tlast(b_ip_pl_tlast), .m_ip_payload_axis_tuser(b_ip_pl_tuser),
    .busy(), .error_payload_early_termination()
);

ip_eth_tx_512 #(.DATA_WIDTH(DW)) i_tx (
    .clk(clk), .rst(rst),
    .s_ip_hdr_valid(b_ip_hdr_valid), .s_ip_hdr_ready(b_ip_hdr_ready),
    .s_eth_dest_mac(b_eth_dest_mac), .s_eth_src_mac(b_eth_src_mac), .s_eth_type(b_eth_type),
    .s_ip_dscp(b_ip_dscp), .s_ip_ecn(b_ip_ecn), .s_ip_length(b_ip_length),
    .s_ip_identification(b_ip_identification), .s_ip_flags(b_ip_flags), .s_ip_fragment_offset(b_ip_fragment_offset),
    .s_ip_ttl(b_ip_ttl), .s_ip_protocol(b_ip_protocol),
    .s_ip_source_ip(b_ip_source_ip), .s_ip_dest_ip(b_ip_dest_ip),
    .s_ip_payload_axis_tdata(b_ip_pl_tdata), .s_ip_payload_axis_tkeep(b_ip_pl_tkeep),
    .s_ip_payload_axis_tvalid(b_ip_pl_tvalid), .s_ip_payload_axis_tready(b_ip_pl_tready),
    .s_ip_payload_axis_tlast(b_ip_pl_tlast), .s_ip_payload_axis_tuser(b_ip_pl_tuser),
    .m_eth_hdr_valid(c_eth_hdr_valid), .m_eth_hdr_ready(c_eth_hdr_ready),
    .m_eth_dest_mac(c_eth_dest_mac), .m_eth_src_mac(c_eth_src_mac), .m_eth_type(c_eth_type),
    .m_eth_payload_axis_tdata(c_eth_pl_tdata), .m_eth_payload_axis_tkeep(c_eth_pl_tkeep),
    .m_eth_payload_axis_tvalid(c_eth_pl_tvalid), .m_eth_payload_axis_tready(c_eth_pl_tready),
    .m_eth_payload_axis_tlast(c_eth_pl_tlast), .m_eth_payload_axis_tuser(c_eth_pl_tuser),
    .busy(), .error_payload_early_termination()
);

ip_eth_rx_512 #(.DATA_WIDTH(DW)) i_rx (
    .clk(clk), .rst(rst),
    .s_eth_hdr_valid(c_eth_hdr_valid), .s_eth_hdr_ready(c_eth_hdr_ready),
    .s_eth_dest_mac(c_eth_dest_mac), .s_eth_src_mac(c_eth_src_mac), .s_eth_type(c_eth_type),
    .s_eth_payload_axis_tdata(c_eth_pl_tdata), .s_eth_payload_axis_tkeep(c_eth_pl_tkeep),
    .s_eth_payload_axis_tvalid(c_eth_pl_tvalid), .s_eth_payload_axis_tready(c_eth_pl_tready),
    .s_eth_payload_axis_tlast(c_eth_pl_tlast), .s_eth_payload_axis_tuser(c_eth_pl_tuser),
    .m_ip_hdr_valid(d_ip_hdr_valid), .m_ip_hdr_ready(d_ip_hdr_ready),
    .m_eth_dest_mac(d_eth_dest_mac), .m_eth_src_mac(d_eth_src_mac), .m_eth_type(d_eth_type),
    .m_ip_version(d_ip_version), .m_ip_ihl(d_ip_ihl), .m_ip_dscp(d_ip_dscp), .m_ip_ecn(d_ip_ecn),
    .m_ip_length(d_ip_length), .m_ip_identification(d_ip_identification), .m_ip_flags(d_ip_flags),
    .m_ip_fragment_offset(d_ip_fragment_offset), .m_ip_ttl(d_ip_ttl), .m_ip_protocol(d_ip_protocol),
    .m_ip_header_checksum(d_ip_header_checksum), .m_ip_source_ip(d_ip_source_ip), .m_ip_dest_ip(d_ip_dest_ip),
    .m_ip_payload_axis_tdata(d_ip_pl_tdata), .m_ip_payload_axis_tkeep(d_ip_pl_tkeep),
    .m_ip_payload_axis_tvalid(d_ip_pl_tvalid), .m_ip_payload_axis_tready(d_ip_pl_tready),
    .m_ip_payload_axis_tlast(d_ip_pl_tlast), .m_ip_payload_axis_tuser(d_ip_pl_tuser),
    .busy(), .error_header_early_termination(), .error_payload_early_termination(),
    .error_invalid_header(), .error_invalid_checksum()
);

udp_ip_rx_512 #(.DATA_WIDTH(DW)) u_rx (
    .clk(clk), .rst(rst),
    .s_ip_hdr_valid(d_ip_hdr_valid), .s_ip_hdr_ready(d_ip_hdr_ready),
    .s_eth_dest_mac(d_eth_dest_mac), .s_eth_src_mac(d_eth_src_mac), .s_eth_type(d_eth_type),
    .s_ip_version(d_ip_version), .s_ip_ihl(d_ip_ihl), .s_ip_dscp(d_ip_dscp), .s_ip_ecn(d_ip_ecn),
    .s_ip_length(d_ip_length), .s_ip_identification(d_ip_identification), .s_ip_flags(d_ip_flags),
    .s_ip_fragment_offset(d_ip_fragment_offset), .s_ip_ttl(d_ip_ttl), .s_ip_protocol(d_ip_protocol),
    .s_ip_header_checksum(d_ip_header_checksum), .s_ip_source_ip(d_ip_source_ip), .s_ip_dest_ip(d_ip_dest_ip),
    .s_ip_payload_axis_tdata(d_ip_pl_tdata), .s_ip_payload_axis_tkeep(d_ip_pl_tkeep),
    .s_ip_payload_axis_tvalid(d_ip_pl_tvalid), .s_ip_payload_axis_tready(d_ip_pl_tready),
    .s_ip_payload_axis_tlast(d_ip_pl_tlast), .s_ip_payload_axis_tuser(d_ip_pl_tuser),
    .m_udp_hdr_valid(e_udp_hdr_valid), .m_udp_hdr_ready(e_udp_hdr_ready),
    .m_eth_dest_mac(), .m_eth_src_mac(), .m_eth_type(),
    .m_ip_version(), .m_ip_ihl(), .m_ip_dscp(), .m_ip_ecn(),
    .m_ip_length(), .m_ip_identification(), .m_ip_flags(), .m_ip_fragment_offset(),
    .m_ip_ttl(), .m_ip_protocol(), .m_ip_header_checksum(),
    .m_ip_source_ip(e_ip_source_ip), .m_ip_dest_ip(e_ip_dest_ip),
    .m_udp_source_port(e_udp_source_port), .m_udp_dest_port(e_udp_dest_port),
    .m_udp_length(e_udp_length), .m_udp_checksum(e_udp_checksum),
    .m_udp_payload_axis_tdata(e_udp_pl_tdata), .m_udp_payload_axis_tkeep(e_udp_pl_tkeep),
    .m_udp_payload_axis_tvalid(e_udp_pl_tvalid), .m_udp_payload_axis_tready(e_udp_pl_tready),
    .m_udp_payload_axis_tlast(e_udp_pl_tlast), .m_udp_payload_axis_tuser(e_udp_pl_tuser),
    .busy(), .error_header_early_termination(), .error_payload_early_termination()
);

reg [7:0] pbuf [0:9215];
reg [7:0] gotbuf [0:9215];
integer plen, got_len, errors = 0;

task send;
    input integer payload_len;
    integer pos,j,nbytes,i;
    reg [DW-1:0] d; reg [KW-1:0] k;
    begin
        plen = payload_len;
        for (i=0;i<payload_len;i=i+1) pbuf[i] = (i*3 + 8'h05) & 8'hff;
        @(posedge clk);
        a_udp_length <= 8 + payload_len;
        a_udp_hdr_valid <= 1;
        @(posedge clk);
        while (!a_udp_hdr_ready) @(posedge clk);
        a_udp_hdr_valid <= 0;
        pos=0;
        while (pos < payload_len) begin
            d=0;k=0; nbytes=payload_len-pos; if (nbytes>KW) nbytes=KW;
            for (j=0;j<nbytes;j=j+1) begin d[j*8+:8]=pbuf[pos+j]; k[j]=1'b1; end
            a_pl_tdata <= d; a_pl_tkeep <= k; a_pl_tvalid <= 1;
            a_pl_tlast <= (pos+nbytes>=payload_len);
            @(posedge clk);
            while (!a_pl_tready) @(posedge clk);
            pos=pos+nbytes;
        end
        a_pl_tvalid <= 0; a_pl_tlast <= 0;
    end
endtask

always @(posedge clk) begin
    if (rst) got_len <= 0;
    else if (e_udp_pl_tvalid && e_udp_pl_tready) begin : col
        integer b;
        for (b=0;b<KW;b=b+1)
            if (e_udp_pl_tkeep[b]) begin gotbuf[got_len]=e_udp_pl_tdata[b*8+:8]; got_len=got_len+1; end
    end
end

task run_case;
    input [127:0] name;
    input integer pl;
    integer i;
    begin
        @(posedge clk);
        got_len = 0;
        send(pl);
        repeat (pl/32 + 80) @(posedge clk);
        if (got_len !== pl) begin
            $display("FAIL[%0s]: rx payload len %0d exp %0d", name, got_len, pl); errors=errors+1;
        end else begin
            for (i=0;i<pl;i=i+1)
                if (gotbuf[i] !== pbuf[i]) begin
                    $display("FAIL[%0s]: byte %0d=%h exp %h", name, i, gotbuf[i], pbuf[i]); errors=errors+1;
                end
            if (e_udp_source_port!==16'h1234 || e_udp_dest_port!==16'd1234) begin
                $display("FAIL[%0s]: ports %h/%h", name, e_udp_source_port, e_udp_dest_port); errors=errors+1;
            end
            if (errors==0) $display("PASS[%0s]: %0d bytes round-tripped, ports/len OK", name, pl);
        end
    end
endtask

initial begin
    repeat (8) @(posedge clk);
    rst <= 0; @(posedge clk);

    run_case("L1",    1);
    run_case("L18",   18);
    run_case("L36",   36);
    run_case("L64",   64);
    run_case("L100",  100);
    run_case("L512",  512);
    run_case("L1472", 1472);
    run_case("L8200", 8200);
    run_case("L8224", 8224);
    run_case("L8960", 8960);

    backpressure = 1;
    run_case("Lbp300",  300);
    run_case("Lbp1472", 1472);
    run_case("Lbp8960", 8960);
    backpressure = 0;

    if (errors==0) $display("\n*** LOOPBACK ALL PASSED ***");
    else           $display("\n*** %0d ERRORS ***", errors);
    $finish;
end

initial begin #20000000 $display("TIMEOUT"); $finish; end

endmodule

`default_nettype wire
