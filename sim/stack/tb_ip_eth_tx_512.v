`timescale 1ns / 1ps
`default_nettype none

// Self-checking testbench for ip_eth_tx_512
module tb_ip_eth_tx_512;

localparam DATA_WIDTH = 512;
localparam KEEP_WIDTH = DATA_WIDTH/8;

reg clk = 0, rst = 1;
always #2.5 clk = ~clk;

reg         s_ip_hdr_valid = 0;
wire        s_ip_hdr_ready;
reg  [15:0] s_ip_length = 0;
reg  [15:0] s_ip_identification = 16'hABCD;
reg  [7:0]  s_ip_ttl = 8'd64;
reg  [7:0]  s_ip_protocol = 8'd17;
reg  [31:0] s_ip_source_ip = 32'hC0A80482;
reg  [31:0] s_ip_dest_ip = 32'hC0A80480;
reg  [DATA_WIDTH-1:0] s_ip_payload_axis_tdata = 0;
reg  [KEEP_WIDTH-1:0] s_ip_payload_axis_tkeep = 0;
reg         s_ip_payload_axis_tvalid = 0;
wire        s_ip_payload_axis_tready;
reg         s_ip_payload_axis_tlast = 0;

wire        m_eth_hdr_valid;
reg         m_eth_hdr_ready = 1;
wire [DATA_WIDTH-1:0] m_eth_payload_axis_tdata;
wire [KEEP_WIDTH-1:0] m_eth_payload_axis_tkeep;
wire        m_eth_payload_axis_tvalid;
reg         m_eth_payload_axis_tready = 1;
reg         backpressure = 0;
reg  [15:0] lfsr = 16'h1234;
always @(posedge clk) begin
    lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
    m_eth_payload_axis_tready <= backpressure ? lfsr[0] : 1'b1;
end
wire        m_eth_payload_axis_tlast;
wire        m_eth_payload_axis_tuser;

ip_eth_tx_512 #(.DATA_WIDTH(DATA_WIDTH)) dut (
    .clk(clk), .rst(rst),
    .s_ip_hdr_valid(s_ip_hdr_valid), .s_ip_hdr_ready(s_ip_hdr_ready),
    .s_eth_dest_mac(48'h020000000000), .s_eth_src_mac(48'hDAD1D2D3D4D5), .s_eth_type(16'h0800),
    .s_ip_dscp(6'd0), .s_ip_ecn(2'd0), .s_ip_length(s_ip_length),
    .s_ip_identification(s_ip_identification), .s_ip_flags(3'b010), .s_ip_fragment_offset(13'd0),
    .s_ip_ttl(s_ip_ttl), .s_ip_protocol(s_ip_protocol),
    .s_ip_source_ip(s_ip_source_ip), .s_ip_dest_ip(s_ip_dest_ip),
    .s_ip_payload_axis_tdata(s_ip_payload_axis_tdata),
    .s_ip_payload_axis_tkeep(s_ip_payload_axis_tkeep),
    .s_ip_payload_axis_tvalid(s_ip_payload_axis_tvalid),
    .s_ip_payload_axis_tready(s_ip_payload_axis_tready),
    .s_ip_payload_axis_tlast(s_ip_payload_axis_tlast),
    .s_ip_payload_axis_tuser(1'b0),
    .m_eth_hdr_valid(m_eth_hdr_valid), .m_eth_hdr_ready(m_eth_hdr_ready),
    .m_eth_dest_mac(), .m_eth_src_mac(), .m_eth_type(),
    .m_eth_payload_axis_tdata(m_eth_payload_axis_tdata),
    .m_eth_payload_axis_tkeep(m_eth_payload_axis_tkeep),
    .m_eth_payload_axis_tvalid(m_eth_payload_axis_tvalid),
    .m_eth_payload_axis_tready(m_eth_payload_axis_tready),
    .m_eth_payload_axis_tlast(m_eth_payload_axis_tlast),
    .m_eth_payload_axis_tuser(m_eth_payload_axis_tuser),
    .busy(), .error_payload_early_termination()
);

reg [7:0] pbuf [0:2047];   // IP payload bytes (input)
reg [7:0] expbuf [0:2047]; // expected eth payload (header+payload)
integer plen, exp_len;
integer errors = 0;

task build;
    input integer payload_len;
    integer i, sum;
    reg [15:0] csum, iplen;
    begin
        plen = payload_len;
        iplen = 20 + payload_len;
        // expected header
        expbuf[0]=8'h45; expbuf[1]=8'h00;
        expbuf[2]=iplen[15:8]; expbuf[3]=iplen[7:0];
        expbuf[4]=8'hAB; expbuf[5]=8'hCD;
        expbuf[6]=8'h40; expbuf[7]=8'h00;
        expbuf[8]=8'd64; expbuf[9]=8'd17;
        expbuf[10]=8'h00; expbuf[11]=8'h00;
        expbuf[12]=8'd192; expbuf[13]=8'd168; expbuf[14]=8'd4; expbuf[15]=8'd130;
        expbuf[16]=8'd192; expbuf[17]=8'd168; expbuf[18]=8'd4; expbuf[19]=8'd128;
        sum=0;
        for (i=0;i<10;i=i+1) sum = sum + ((expbuf[2*i]<<8)|expbuf[2*i+1]);
        sum=(sum&16'hffff)+(sum>>16); sum=(sum&16'hffff)+(sum>>16);
        csum=~sum[15:0];
        expbuf[10]=csum[15:8]; expbuf[11]=csum[7:0];
        for (i=0;i<payload_len;i=i+1) begin
            pbuf[i]=(i+8'h31)&8'hff;
            expbuf[20+i]=(i+8'h31)&8'hff;
        end
        exp_len = 20 + payload_len;
    end
endtask

task send;
    integer pos,j,nbytes;
    reg [DATA_WIDTH-1:0] d; reg [KEEP_WIDTH-1:0] k;
    begin
        @(posedge clk);
        s_ip_length <= 20 + plen;
        s_ip_hdr_valid <= 1;
        @(posedge clk);
        while (!s_ip_hdr_ready) @(posedge clk);
        s_ip_hdr_valid <= 0;
        pos=0;
        while (pos < plen) begin
            d=0;k=0; nbytes=plen-pos; if (nbytes>KEEP_WIDTH) nbytes=KEEP_WIDTH;
            for (j=0;j<nbytes;j=j+1) begin d[j*8+:8]=pbuf[pos+j]; k[j]=1'b1; end
            s_ip_payload_axis_tdata <= d;
            s_ip_payload_axis_tkeep <= k;
            s_ip_payload_axis_tvalid <= 1;
            s_ip_payload_axis_tlast <= (pos+nbytes>=plen);
            @(posedge clk);
            while (!s_ip_payload_axis_tready) @(posedge clk);
            pos=pos+nbytes;
        end
        s_ip_payload_axis_tvalid <= 0;
        s_ip_payload_axis_tlast <= 0;
    end
endtask

reg [7:0] gotbuf [0:2047];
integer got_len;
always @(posedge clk) begin
    if (rst) got_len <= 0;
    else if (m_eth_payload_axis_tvalid && m_eth_payload_axis_tready) begin : col
        integer b;
        for (b=0;b<KEEP_WIDTH;b=b+1)
            if (m_eth_payload_axis_tkeep[b]) begin gotbuf[got_len]=m_eth_payload_axis_tdata[b*8+:8]; got_len=got_len+1; end
    end
end

task run_case;
    input [127:0] name;
    input integer pl;
    integer i;
    begin
        @(posedge clk);
        got_len=0;
        build(pl);
        send;
        repeat (60) @(posedge clk);
        if (got_len !== exp_len) begin
            $display("FAIL[%0s]: eth payload len %0d exp %0d", name, got_len, exp_len); errors=errors+1;
        end else begin
            for (i=0;i<exp_len;i=i+1)
                if (gotbuf[i]!==expbuf[i]) begin
                    $display("FAIL[%0s]: byte %0d=%h exp %h", name, i, gotbuf[i], expbuf[i]); errors=errors+1;
                end
            if (errors==0) $display("PASS[%0s]: %0d eth bytes OK (hdr+payload, csum verified)", name, exp_len);
        end
    end
endtask

initial begin
    repeat (8) @(posedge clk);
    rst <= 0; @(posedge clk);

    run_case("t1",    1);
    run_case("t44",   44);
    run_case("t45",   45);
    run_case("t64",   64);
    run_case("t100",  100);
    run_case("t1472", 1472);

    backpressure = 1;
    run_case("bp100", 100);
    run_case("bp1472",1472);
    backpressure = 0;

    if (errors==0) $display("\n*** ALL TESTS PASSED ***");
    else           $display("\n*** %0d ERRORS ***", errors);
    $finish;
end

integer dbg;
initial begin
    @(negedge rst);
    for (dbg=0; dbg<30; dbg=dbg+1) begin
        @(posedge clk);
        $display("t=%0t st=%0d s_hdr_v=%b s_hdr_r=%b s_pl_v=%b s_pl_r=%b m_pl_v=%b m_pl_r=%b m_last=%b wc=%0d",
            $time, dut.state_reg, s_ip_hdr_valid, s_ip_hdr_ready,
            s_ip_payload_axis_tvalid, s_ip_payload_axis_tready,
            m_eth_payload_axis_tvalid, m_eth_payload_axis_tready, m_eth_payload_axis_tlast,
            dut.word_count_reg);
    end
end

initial begin #400000 $display("TIMEOUT"); $finish; end

endmodule

`default_nettype wire
