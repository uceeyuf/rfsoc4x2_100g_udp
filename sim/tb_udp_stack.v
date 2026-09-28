`timescale 1ns / 1ps
// System test of udp_stack (verilog-ethernet + icmp_echo + udp_echo) at the Ethernet
// frame level: ARP, ICMP echo, UDP echo on two ports under bursts and MAC back-pressure,
// UDP echo while the peer MAC is still unresolved, alias addresses (ARP, echo replies from them),
// and the record stream (record_eth_tx merged in front of the MAC as in fpga_core): headers,
// sequence, data and throughput.
module tb_udp_stack;
    localparam DW = 512, KW = DW / 8;
    reg clk = 0, rst = 1;
    always #2.5 clk = ~clk;

    localparam [47:0] FPGA_MAC = 48'h02_00_00_00_00_00;
    localparam [31:0] FPGA_IP  = {8'd192, 8'd168, 8'd100, 8'd1};
    localparam [31:0] ALIAS_IP = {8'd192, 8'd168, 8'd100, 8'd128};   // 32 alias addresses
    localparam [47:0] PC_MAC   = 48'hEC_0D_9A_44_D8_8C;
    localparam [31:0] PC_IP    = {8'd192, 8'd168, 8'd100, 8'd2};

    // ---------------------------------------------------------------- DUT
    reg  [DW-1:0] rx_tdata = 0; reg [KW-1:0] rx_tkeep = 0; reg rx_tvalid = 0, rx_tlast = 0;
    wire        rx_tready;
    wire [DW-1:0] tx_tdata; wire [KW-1:0] tx_tkeep; wire tx_tvalid, tx_tlast, tx_tuser;
    reg         tx_tready = 1;

    wire [DW-1:0] sk_tdata; wire [KW-1:0] sk_tkeep; wire sk_tvalid, sk_tready, sk_tlast, sk_tuser;
    wire [DW-1:0] rs_tdata; wire rs_tvalid, rs_tready, rs_tlast, rs_tuser;
    wire [47:0] stream_dest_mac;
    wire [31:0] stream_dest_ip;

    udp_stack #(.DATA_WIDTH(DW), .ALIAS_BASE(ALIAS_IP), .ALIAS_COUNT(32)) dut (
        .clk(clk), .rst(rst),
        .local_mac(FPGA_MAC), .local_ip(FPGA_IP), .gateway_ip({8'd192, 8'd168, 8'd100, 8'd254}),
        .subnet_mask(32'hFFFFFF00),
        .rx_axis_tdata(rx_tdata), .rx_axis_tkeep(rx_tkeep), .rx_axis_tvalid(rx_tvalid),
        .rx_axis_tready(rx_tready), .rx_axis_tlast(rx_tlast), .rx_axis_tuser(1'b0),
        .tx_axis_tdata(sk_tdata), .tx_axis_tkeep(sk_tkeep), .tx_axis_tvalid(sk_tvalid),
        .tx_axis_tready(sk_tready), .tx_axis_tlast(sk_tlast), .tx_axis_tuser(sk_tuser),
        .stream_dest_mac(stream_dest_mac), .stream_dest_ip(stream_dest_ip)
    );

    // record stream: source = running sample index (16 samples per beat), wraps with the record
    localparam TB_SAMPLES = 16 * 128 * 6;
    reg         st_trig = 0, st_jumbo = 0, st_vary_ip = 0, src_random = 0;
    reg  [4:0]  st_flows = 0;
    reg  [15:0] st_gap = 0;
    reg  [31:0] src_idx = 0;
    reg         src_v = 0;
    wire        src_r;
    wire [3:0]  st_state;
    reg  [DW-1:0] src_d;
    integer     si;
    always @* for (si = 0; si < 16; si = si + 1) src_d[si*32 +: 32] = src_idx + si;
    always @(posedge clk) begin
        if (src_v && src_r) src_idx <= (src_idx + 16 >= TB_SAMPLES) ? 0 : src_idx + 16;
        if (!src_v || src_r) src_v <= src_random ? (($random & 3) != 0) : 1'b1;
    end

    record_eth_tx #(.TOTAL_SAMPLES(TB_SAMPLES)) stream (
        .clk(clk), .rst(rst),
        .local_mac(FPGA_MAC), .local_ip(FPGA_IP), .local_port(16'd1236),
        .dest_mac(stream_dest_mac), .dest_ip(stream_dest_ip), .dest_port(16'd1237),
        .trig(st_trig), .gap_cycles(st_gap), .flow_count(st_flows), .flow_vary_ip(st_vary_ip),
        .jumbo(st_jumbo), .txstate(st_state),
        .s_axis_tdata(src_d), .s_axis_tvalid(src_v), .s_axis_tready(src_r),
        .m_axis_tdata(rs_tdata), .m_axis_tkeep(), .m_axis_tvalid(rs_tvalid), .m_axis_tready(rs_tready),
        .m_axis_tlast(rs_tlast), .m_axis_tuser(rs_tuser)
    );

    axis_arb_mux #(
        .S_COUNT(2), .DATA_WIDTH(DW), .KEEP_ENABLE(1), .KEEP_WIDTH(KW), .ID_ENABLE(0), .DEST_ENABLE(0),
        .USER_ENABLE(1), .USER_WIDTH(1), .LAST_ENABLE(1), .ARB_TYPE_ROUND_ROBIN(1), .ARB_LSB_HIGH_PRIORITY(1)
    ) tx_mux (
        .clk(clk), .rst(rst),
        .s_axis_tdata({rs_tdata, sk_tdata}), .s_axis_tkeep({{KW{1'b1}}, sk_tkeep}),
        .s_axis_tvalid({rs_tvalid, sk_tvalid}), .s_axis_tready({rs_tready, sk_tready}),
        .s_axis_tlast({rs_tlast, sk_tlast}), .s_axis_tid(16'd0), .s_axis_tdest(16'd0),
        .s_axis_tuser({rs_tuser, sk_tuser}),
        .m_axis_tdata(tx_tdata), .m_axis_tkeep(tx_tkeep), .m_axis_tvalid(tx_tvalid), .m_axis_tready(tx_tready),
        .m_axis_tlast(tx_tlast), .m_axis_tid(), .m_axis_tdest(), .m_axis_tuser(tx_tuser)
    );

    integer errors = 0;
    task fail(input [8*80-1:0] msg); begin errors = errors + 1; $display("ERROR: %0s", msg); end endtask

    // ---------------------------------------------------------------- frame builder
    reg [7:0] fb [0:9215];
    integer   flen;

    task put16(input integer o, input [15:0] v); begin fb[o] = v[15:8]; fb[o+1] = v[7:0]; end endtask
    task put32(input integer o, input [31:0] v); begin put16(o, v[31:16]); put16(o+2, v[15:0]); end endtask
    task put48(input integer o, input [47:0] v); begin put16(o, v[47:32]); put32(o+2, v[31:0]); end endtask

    function [15:0] csum(input integer start, input integer n);   // over fb[start..start+n-1]
        integer i; reg [31:0] s;
        begin
            s = 0;
            for (i = 0; i < n; i = i + 2)
                s = s + {fb[start+i], (i+1 < n) ? fb[start+i+1] : 8'h00};
            while (s[31:16]) s = s[15:0] + s[31:16];
            csum = ~s[15:0];
        end
    endfunction

    task eth_hdr(input [15:0] ethertype);
        begin put48(0, FPGA_MAC); put48(6, PC_MAC); put16(12, ethertype); end
    endtask

    reg [31:0] dst_ip = FPGA_IP;      // destination of the frames built below
    reg [31:0] last_arp_spa = 0;

    task ip_hdr(input [15:0] total_len, input [7:0] proto);
        begin
            fb[14] = 8'h45; fb[15] = 0; put16(16, total_len); put16(18, 16'h1234); put16(20, 16'h4000);
            fb[22] = 64; fb[23] = proto; put16(24, 0); put32(26, PC_IP); put32(30, dst_ip);
            put16(24, csum(14, 20));
        end
    endtask

    task build_udp(input [15:0] sport, input [15:0] dport, input integer plen, input integer seed);
        integer i;
        begin
            eth_hdr(16'h0800); ip_hdr(28 + plen, 8'd17);
            put16(34, sport); put16(36, dport); put16(38, 8 + plen); put16(40, 0);
            for (i = 0; i < plen; i = i + 1) fb[42+i] = (seed * 7 + i * 13) & 8'hff;
            flen = 42 + plen;
        end
    endtask

    task build_arp(input [15:0] oper, input [47:0] tha, input [31:0] tpa);
        begin
            put48(0, oper == 1 ? 48'hFFFFFFFFFFFF : FPGA_MAC); put48(6, PC_MAC); put16(12, 16'h0806);
            put16(14, 1); put16(16, 16'h0800); fb[18] = 6; fb[19] = 4; put16(20, oper);
            put48(22, PC_MAC); put32(28, PC_IP); put48(32, tha); put32(38, tpa);
            flen = 42;
        end
    endtask

    task build_ping(input [15:0] seq, input integer dlen);
        integer i;
        begin
            eth_hdr(16'h0800); ip_hdr(28 + dlen, 8'd1);
            fb[34] = 8; fb[35] = 0; put16(36, 0); put16(38, 16'h0001); put16(40, seq);
            for (i = 0; i < dlen; i = i + 1) fb[42+i] = 8'h61 + (i % 23);
            put16(36, csum(34, 8 + dlen));
            flen = 42 + dlen;
        end
    endtask

    // push fb[0..flen-1] into the DUT at no more than 100G line rate (clk = 200 MHz);
    // counts cycles spent waiting for rx_tready
    integer rx_stall;
    task send_frame;
        integer pos, j;
        reg [DW-1:0] d; reg [KW-1:0] kp;
        begin
            pos = 0;
            while (pos < flen) begin
                d = 0; kp = 0;
                for (j = 0; j < KW; j = j + 1)
                    if (pos + j < flen) begin d[j*8 +: 8] = fb[pos+j]; kp[j] = 1'b1; end
                rx_tdata <= d; rx_tkeep <= kp;
                rx_tvalid <= 1; rx_tlast <= (pos + KW >= flen);
                @(posedge clk);
                while (!rx_tready) begin rx_stall = rx_stall + 1; @(posedge clk); end
                pos = pos + KW;
            end
            rx_tvalid <= 0; rx_tlast <= 0;
            // at most ~95 Gbps on the wire (frame + FCS + preamble + IFG): the 512-bit TX path adds
            // ~7 idle cycles per frame at 200 MHz, so an echo cannot keep up with a full 100G input
            repeat ((((flen + 24) * 17 + 999) / 1000) - ((flen + KW - 1) / KW)) @(posedge clk);
        end
    endtask

    // ---------------------------------------------------------------- TX monitor
    reg [7:0]  ob [0:9215];
    integer    olen = 0;
    integer    n_arp_req = 0, n_arp_rep = 0, n_ping = 0, n_echo = 0, n_stream = 0, n_flow = 0;
    reg [15:0] exp_len [0:1023];      // expected echo payload length, per sequence number
    reg [7:0]  last_seed;
    integer    echo_ok = 0;

    function [15:0] g16(input integer o); g16 = {ob[o], ob[o+1]}; endfunction
    function [31:0] le32(input integer o); le32 = {ob[o+3], ob[o+2], ob[o+1], ob[o]}; endfunction

    // record stream: headers, flow rotation, running start index and sample data
    integer exp_si = 0, exp_flow = 0, stream_err = 0;
    task check_stream;
        integer j, fl, si, nd, e0;
        begin
            n_stream = n_stream + 1;
            e0 = errors;
            fl = g16(34) - 1236;
            si = le32(42);
            nd = (olen - 64) / 4;
            if ({ob[0], ob[1], ob[2], ob[3], ob[4], ob[5]} != PC_MAC) fail("stream destination MAC");
            if ({ob[6], ob[7], ob[8], ob[9], ob[10], ob[11]} != FPGA_MAC) fail("stream source MAC");
            if (g16(36) != 1237 + fl) fail("stream destination port");
            if (fl != exp_flow) fail("stream flow order");
            if ({g16(26), g16(28)} != FPGA_IP + (st_vary_ip ? fl : 0)) fail("stream source IP");
            if ({g16(30), g16(32)} != PC_IP) fail("stream destination IP");
            if (g16(16) + 14 != olen) fail("stream IP length");
            if (g16(38) != g16(16) - 20) fail("stream UDP length");
            if (olen != 64 + 64 * (st_jumbo ? 128 : 16)) fail("stream frame length");
            if (le32(46) != TB_SAMPLES) fail("stream total samples");
            if (si != exp_si) fail("stream start index");
            for (j = 0; j < nd; j = j + 1)
                if (le32(64 + 4 * j) != si + j) begin fail("stream data"); j = nd; end
            exp_si   = (si + nd) % TB_SAMPLES;
            exp_flow = (fl + 1) % (st_flows > 1 ? st_flows : 1);
            if (errors != e0) begin
                stream_err = stream_err + 1;
                $display("  stream frame %0d: len %0d flow %0d start %0d", n_stream, olen, fl, si);
            end
        end
    endtask

    task check_frame;
        integer i, plen, seed, ihl;
        reg [31:0] s;
        begin
            if (g16(12) == 16'h0806) begin
                if (g16(20) == 1) n_arp_req = n_arp_req + 1; else begin n_arp_rep = n_arp_rep + 1; last_arp_spa = {g16(28), g16(30)}; end
                if (olen < 60) begin $display("  ARP frame %0d bytes", olen); end
            end else if (g16(12) == 16'h0800) begin
                // verify IP header checksum
                s = 0; for (i = 14; i < 34; i = i + 2) s = s + g16(i);
                while (s[31:16]) s = s[15:0] + s[31:16];
                if (s[15:0] != 16'hFFFF) fail("IP header checksum");
                if (ob[23] == 1) begin
                    n_ping = n_ping + 1;
                    if (ob[34] != 0) fail("ICMP type is not echo reply");
                    s = 0; for (i = 34; i < 14 + g16(16); i = i + 2) s = s + {ob[i], (i + 1 < 14 + g16(16)) ? ob[i+1] : 8'h00};
                    while (s[31:16]) s = s[15:0] + s[31:16];
                    if (s[15:0] != 16'hFFFF) fail("ICMP checksum");
                end else if (ob[23] == 17) begin
                    plen = g16(38) - 8;
                    if (g16(34) >= 16'd1236 && g16(34) < 16'd1236 + 32 && g16(36) == g16(34) + 1) begin
                        check_stream;
                    end else begin
                        n_echo = n_echo + 1;
                        if (g16(36) >= 16'd6000 && g16(36) < 16'd6016) begin
                            // flow echo: sent to alias address ALIAS_IP + p, the reply must come from it
                            n_flow = n_flow + 1;
                            if (g16(34) != 16'd1234) fail("flow echo source port");
                            if ({g16(26), g16(28)} != ALIAS_IP + (g16(36) - 16'd6000)) fail("flow echo source IP");
                        end else begin
                            if (g16(36) != 16'd5000 && g16(36) != 16'd5001) fail("echo destination port");
                            if (g16(34) != (g16(36) == 16'd5000 ? 16'd1234 : 16'd1235)) fail("echo source port");
                            if ({g16(26), g16(28)} != FPGA_IP) fail("echo source IP");
                        end
                        // payload was (seed*7 + i*13); seed is in the source port pairing, recover from byte 0
                        seed = -1;
                        for (i = 0; i < 256; i = i + 1) if (((i * 7) & 8'hff) == ob[42]) seed = i;
                        for (i = 0; i < plen; i = i + 1)
                            if (ob[42+i] != ((seed * 7 + i * 13) & 8'hff)) begin
                                begin $display("  payload mismatch: udp_len=%0d frame=%0d sport=%0d byte %0d", g16(38), olen, g16(34), i); fail("echo payload"); i = plen; end
                            end
                        if (olen != 42 + plen && !(plen < 18 && olen == 60)) fail("echo frame length");
                        echo_ok = echo_ok + 1;
                    end
                end
            end
        end
    endtask

    integer k;
    always @(posedge clk) if (!rst && tx_tvalid && tx_tready) begin
        for (k = 0; k < KW; k = k + 1) if (tx_tkeep[k]) begin ob[olen] = tx_tdata[k*8 +: 8]; olen = olen + 1; end
        if (tx_tlast) begin
            if (tx_tuser) begin $display("  bad frame: %0d bytes udp_len=%0d", olen, g16(38)); fail("frame marked bad on TX"); end
            check_frame;
            olen = 0;
        end
    end

    // random MAC back-pressure when enabled
    reg backpressure = 0;
    always @(posedge clk) tx_tready <= backpressure ? ($random & 1) : 1'b1;

    // ---------------------------------------------------------------- stream source
    // stream `frames` frames and stop; returns the cycles from the first to the last stream beat
    integer stream_cycles;
    task run_stream(input integer frames, input jumbo, input [4:0] flows, input vary_ip);
        integer n0s, tfirst;
        begin
            wait (st_state == 0);
            src_idx = 0; exp_si = 0; exp_flow = 0;
            st_jumbo = jumbo; st_flows = flows; st_vary_ip = vary_ip;
            n0s = n_stream;
            @(posedge clk); st_trig <= 1;
            @(posedge clk); while (!(rs_tvalid && rs_tready)) @(posedge clk);
            tfirst = $time;
            while (n_stream - n0s < frames - 1) @(posedge clk);
            st_trig <= 0;
            wait (st_state == 4);
            repeat (200) @(posedge clk);
            stream_cycles = ($time - tfirst) / 5;
        end
    endtask

    // cycles per frame over the stream beats handed to the MAC while it takes everything
    integer rate_beats = 0, rate_cycles = 0, rate_on = 0;
    always @(posedge clk) if (rate_on) begin
        rate_cycles = rate_cycles + 1;
        if (tx_tvalid && tx_tready) rate_beats = rate_beats + 1;
    end

    // ---------------------------------------------------------------- test sequence
    integer p, n0, t0;

    task burst(input integer count);
        integer q;
        begin
            for (q = 0; q < count; q = q + 1) begin
                build_udp(q[0] ? 16'd5001 : 16'd5000, q[0] ? 16'd1235 : 16'd1234,
                          (q % 5 == 0) ? 1 : (q % 7 == 0) ? 8192 : (q % 3 == 0) ? 1472 : 64 + q * 9,
                          (q % 200) + 3);
                send_frame;
            end
        end
    endtask

    task wait_echo(input integer base, input integer count);   // until all arrived or 1 ms idle
        integer last, idle;
        begin
            last = echo_ok; idle = 0;
            while (echo_ok - base < count && idle < 200000) begin
                @(posedge clk);
                if (echo_ok != last) begin last = echo_ok; idle = 0; end else idle = idle + 1;
            end
        end
    endtask
    initial begin
        rx_stall = 0;
        repeat (20) @(posedge clk); rst <= 0; repeat (20) @(posedge clk);

        // 1) UDP echo while the FPGA does not know our MAC: it must ARP for us, and the
        //    receive path must keep accepting frames meanwhile
        build_udp(16'd5000, 16'd1234, 100, 1); send_frame;
        build_udp(16'd5001, 16'd1235, 200, 2); send_frame;
        t0 = $time;
        wait (n_arp_req > 0);
        if (rx_stall > 1000) fail("receive path blocked while the peer MAC was unresolved");
        build_arp(16'd2, FPGA_MAC, FPGA_IP); send_frame;     // ARP reply to the FPGA
        wait (echo_ok >= 2);
        $display("PASS 1: echo with ARP resolution (ARP requests %0d, rx stall cycles %0d)", n_arp_req, rx_stall);

        // 2) ARP request for the FPGA
        n0 = n_arp_rep;
        build_arp(16'd1, 48'd0, FPGA_IP); send_frame;
        wait (n_arp_rep > n0);
        $display("PASS 2: ARP reply");

        // 3) ping: 32-byte (Windows default) and a 1000-byte request
        build_ping(16'd1, 32); send_frame;
        build_ping(16'd2, 1000); send_frame;
        wait (n_ping >= 2);
        $display("PASS 3: ICMP echo replies with valid checksums");

        // 4) burst: 120 back-to-back packets to 1234/1235 incl. tiny and jumbo; the MAC takes
        //    everything -> every packet must come back
        n0 = echo_ok;
        burst(120);
        wait_echo(n0, 120);
        if (echo_ok - n0 != 120) fail("not every burst packet was echoed");
        $display("PASS 4: %0d/120 burst packets echoed byte-exact", echo_ok - n0);

        // 5) same burst with random MAC back-pressure (TX slower than RX): the echo may drop
        //    whole packets when its FIFO is full, but never corrupts or stalls
        backpressure = 1;
        n0 = echo_ok;
        burst(120);
        wait_echo(n0, 120);
        backpressure = 0;
        $display("PASS 5: overload: %0d/120 echoed, all byte-exact (rest dropped as whole packets)", echo_ok - n0);

        // 6) the path is still healthy afterwards
        n0 = echo_ok;
        burst(20);
        wait_echo(n0, 20);
        if (echo_ok - n0 != 20) fail("echo path not healthy after overload");
        else $display("PASS 6: 20/20 echoed after the overload");
        if (stream_dest_ip != PC_IP) fail("stream destination IP not latched");
        if (stream_dest_mac != PC_MAC) fail("stream destination MAC not latched");

        // alias addresses: ARP for one is answered with it as sender; 16 jumbo packets to
        // ALIAS_IP + p come back from ALIAS_IP + p; an address outside the block is not echoed
        n0 = n_arp_rep;
        build_arp(16'd1, 48'd0, ALIAS_IP + 12); send_frame;
        wait (n_arp_rep > n0);
        if (last_arp_spa != ALIAS_IP + 12) fail("ARP reply for an alias address");
        n0 = echo_ok;
        for (p = 0; p < 16; p = p + 1) begin dst_ip = ALIAS_IP + p; build_udp(16'd6000 + p, 16'd1234, 8956, p + 50); send_frame; end
        dst_ip = ALIAS_IP + 40; build_udp(16'd6016, 16'd1234, 100, 99); send_frame;
        dst_ip = FPGA_IP;
        wait_echo(n0, 17);
        if (n_flow != 16 || echo_ok - n0 != 16) fail("alias echo");
        else $display("PASS 6b: ARP for an alias address; 16/16 echoes from their alias address, byte-exact; outside the block ignored");

        // 7) record stream, standard frames, 4 flows rotating the source IP, across a record wrap
        n0 = n_stream;
        run_stream(60, 0, 5'd4, 1);
        if (n_stream - n0 < 60 || stream_err) fail("standard stream");
        else $display("PASS 7: %0d standard stream frames (4 flows), headers/sequence/data exact", n_stream - n0);

        // 8) jumbo frames, 8 flows, DDR data arriving with gaps and MAC back-pressure
        src_random = 1; backpressure = 1;
        n0 = n_stream;
        run_stream(20, 1, 5'd8, 0);
        src_random = 0; backpressure = 0;
        if (n_stream - n0 < 20 || stream_err) fail("jumbo stream under back-pressure");
        else $display("PASS 8: %0d jumbo stream frames (8 flows) with source gaps and back-pressure", n_stream - n0);

        // 9) throughput with the MAC and the source always ready, and a ping answered meanwhile
        n0 = n_stream; p = n_ping;
        fork
            run_stream(80, 1, 5'd8, 1);
            begin
                repeat (3000) @(posedge clk);
                rate_on = 1; repeat (129 * 20) @(posedge clk); rate_on = 0;
                build_ping(16'd3, 64); send_frame;
                repeat (2000) @(posedge clk);
            end
        join
        t0 = 0; while (n_ping == p && t0 < 20000) begin @(posedge clk); t0 = t0 + 1; end
        if (n_ping != p + 1) fail("ping during the stream");
        if (stream_err) fail("stream during throughput test");
        $display("PASS 9: jumbo stream %0d beats in %0d cycles = %0d.%02d cycles per 129-beat frame (100G line rate needs <= 132.5)",
                 rate_beats, rate_cycles, rate_cycles * 129 / rate_beats, (rate_cycles * 12900 / rate_beats) % 100);
        if (rate_cycles * 129 > rate_beats * 131) fail("stream slower than 100G line rate");

        if (errors == 0) $display("\n*** udp_stack: ALL TESTS PASSED ***");
        else             $display("\n*** udp_stack: %0d ERRORS ***", errors);
        $finish;
    end

    initial begin #60000000 $display("TIMEOUT (echo_ok=%0d ping=%0d arp_req=%0d)", echo_ok, n_ping, n_arp_req); $finish; end
endmodule
