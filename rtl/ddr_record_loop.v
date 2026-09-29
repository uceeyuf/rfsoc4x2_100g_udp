// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Yijie Yu
//
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: ddr_record_loop
// Description:
//   DDS / 递增数持续写 PL DDR4 + 持续读 DDR4 → 512bit AXIS（喂给 record_eth_tx）
//   纯 PL（无软核）：用 AXI DataMover 命令流驱动，DDR4 MIG 提供 AXI + 物理口。
//
//   两条独立环路（trig 高、calib 完成后一直跑）：
//     写环： DDS(clk_wr=125M) ─pack2→64─ async_fifo_adapter(64@125→512@ui) ─→ S2MM ─AXI写→ DDR(乒乓 2×4MB)
//            写命令FSM(ui_clk): 循环发 (addr, btt=4MB)；每 4MB 流上打一个 tlast
//     读环： DDR ─AXI读→ MM2S ─→ async FIFO(512@ui→512@clk) ─→ m_axis(512@250) → record_eth_tx
//            读命令FSM(ui_clk): 循环发 (addr, btt=4MB)
//
//   MM2S 只用 AXI 读通道、S2MM 只用写通道，二者直接拼到 MIG 单个 S_AXI，无需 Interconnect。
//
//   ★ 时钟域：clk_wr=125M(DDS/写打包，DDS 保持 125 MSps / 5 MHz)，
//     ui_clk=300M(MIG AXI/DataMover，DDR4-2400)，clk=250M(读出 512bit 流 → UDP 发送，512b×250M=128Gbps)。
//     跨域全部经 axis_async_fifo_adapter，单比特电平信号(trig/calib/no_write)用两级 ASYNC_REG 同步。
//     两条命令 FSM 在 ui_clk 域；读命令最多 2 条在途。
//   ★ 带宽：DDR4-2400 峰值 153.6 Gbps；读 ≤128 + 写 4 Gbps。rd/wr_beats_per_s 给出实测握手拍数。
//
//   ★ 注意：读写并发访问同一 4MB ring，DDS 又是周期正弦，读到的段可能新旧交错(撕裂)；
//     对平稳正弦显示无碍，序号(index)对齐仍成立。
//   ★ 本模块复杂，务必先仿真(testbench)验证命令格式/握手/tlast/跨域，再上板。
//
//   样本/线序：64bit = {sample(s+1),sample(s)}，sample={sin[15:0],cos[15:0]}，cos 低位。
//////////////////////////////////////////////////////////////////////////////////

module ddr_record_loop #(
    parameter TOTAL_SAMPLES = 1000000,                 // 整段样本数(32bit/样本)
    parameter BASE_ADDR     = 32'h0000_0000,           // DDR 基址
    parameter TEST_RAMP     = 0,                        // 1:写入"记录内样本序号"递增数(每帧回绕,显示定住)
    parameter IDX_W         = $clog2(TOTAL_SAMPLES),    // 序号位宽
    parameter BTT_BYTES     = TOTAL_SAMPLES*4          // 每命令字节数
)(
    // 读出/以太域（250MHz）
    input  wire         clk,
    input  wire         rst,
    input  wire         trig,           // 高：持续读写；低：停（clk 域）

    // 写入域（DDS / 打包，125MHz）
    input  wire         clk_wr,
    input  wire         rst_wr,

    // DDR4 物理参考时钟 / 复位（来自顶层引脚）
    input  wire         c0_sys_clk_p,
    input  wire         c0_sys_clk_n,
    input  wire         sys_rst,        // MIG 系统复位（顶层按钮 AV12）
    output wire         init_calib_complete,

    // DDR4 物理口（直连顶层引脚）
    output wire [16:0]  c0_ddr4_adr,
    output wire [1:0]   c0_ddr4_ba,
    output wire [0:0]   c0_ddr4_cke,
    output wire [0:0]   c0_ddr4_cs_n,
    inout  wire [7:0]   c0_ddr4_dm_dbi_n,
    inout  wire [63:0]  c0_ddr4_dq,
    inout  wire [7:0]   c0_ddr4_dqs_c,
    inout  wire [7:0]   c0_ddr4_dqs_t,
    output wire [0:0]   c0_ddr4_odt,
    output wire [0:0]   c0_ddr4_bg,
    output wire         c0_ddr4_reset_n,
    output wire         c0_ddr4_act_n,
    output wire [0:0]   c0_ddr4_ck_c,
    output wire [0:0]   c0_ddr4_ck_t,

    // benchmark: 1 = do not write DDR (reads start at once); per-second counts (ui_clk domain)
    input  wire         no_write,
    output wire [31:0]  rd_beats_per_s,      // MM2S R handshakes (512 bit each)
    output wire [31:0]  wr_beats_per_s,      // S2MM W handshakes (512 bit each)

    // 512-bit AXIS output (clk domain) -> record_eth_tx
    output wire [511:0] m_axis_tdata,
    output wire         m_axis_tvalid,
    input  wire         m_axis_tready
);

    localparam [22:0] BTT = BTT_BYTES[22:0];           // 4,000,000 < 2^23 OK

    //=========================================================================
    // MIG 输出时钟/复位
    //=========================================================================
    wire        ui_clk;                 // 300MHz (DDR4-2400)
    wire        ui_clk_sync_rst;        // 高有效
    wire        calib_done;
    assign init_calib_complete = calib_done;

    wire aresetn = ~ui_clk_sync_rst;    // AXI 复位(低有效)，给 MIG.s_axi 与 DataMover

    //=========================================================================
    // 跨域同步：trig / calib
    //=========================================================================
    // 名字以 _cdc_sr 结尾的寄存器由 constraints/cdc.xdc 约束（首级 set_false_path）
    (* ASYNC_REG="true" *) reg [1:0] trig_ui_cdc_sr, calib_ui_cdc_sr, trig_wr_cdc_sr, calib_wr_cdc_sr;
    (* ASYNC_REG="true" *) reg [1:0] nowr_ui_cdc_sr, nowr_wr_cdc_sr;
    always @(posedge ui_clk) nowr_ui_cdc_sr <= {nowr_ui_cdc_sr[0], no_write};
    always @(posedge clk_wr) nowr_wr_cdc_sr <= {nowr_wr_cdc_sr[0], no_write};
    always @(posedge ui_clk) begin
        trig_ui_cdc_sr  <= {trig_ui_cdc_sr[0],  trig};
        calib_ui_cdc_sr <= {calib_ui_cdc_sr[0], calib_done};
    end
    always @(posedge clk_wr) begin
        trig_wr_cdc_sr  <= {trig_wr_cdc_sr[0],  trig};
        calib_wr_cdc_sr <= {calib_wr_cdc_sr[0], calib_done};
    end
    wire go_ui     = trig_ui_cdc_sr[1] & calib_ui_cdc_sr[1];   // ui 域：可读写
    wire go_wr     = trig_wr_cdc_sr[1] & calib_wr_cdc_sr[1] & !nowr_wr_cdc_sr[1];   // 写入域：可写入(pack)
    wire rd_only   = nowr_ui_cdc_sr[1];

    //=========================================================================
    // DDS（clk_wr = 125MHz：dds_compiler_125M_5M 按 125MHz 配置，输出 5MHz）
    //=========================================================================
    wire        dds_tvalid;
    wire [31:0] dds_tdata;
    dds_compiler_125M_5M u_dds (
        .aclk(clk_wr), .m_axis_data_tvalid(dds_tvalid), .m_axis_data_tdata(dds_tdata)
    );

    //=========================================================================
    // 写环 datapath：DDS pack2→64，每 4MB(=500000 个64bit字) 打 tlast
    //=========================================================================
    reg        pack_phase;
    reg [31:0] sample_lo;
    reg [63:0] w64_tdata;
    reg        w64_tvalid;
    reg        w64_tlast;
    reg [20:0] wword_cnt;               // 0..WWORDS-1
    reg [IDX_W-1:0] samp_idx;           // 记录内样本序号(TEST_RAMP用，每帧回绕)
    localparam [20:0] WWORDS = (TOTAL_SAMPLES/2);   // 64bit 字/帧
    wire       w64_tready;

    // 取样：TEST_RAMP=1 用"记录内样本序号"递增数；=0 用 DDS
    wire [31:0] samp_in = TEST_RAMP ? {{(32-IDX_W){1'b0}}, samp_idx} : dds_tdata;

    always @(posedge clk_wr) begin
        if (rst_wr) begin
            pack_phase <= 1'b0; w64_tvalid <= 1'b0; w64_tlast <= 1'b0;
            wword_cnt <= 21'd0; samp_idx <= {IDX_W{1'b0}};
        end else begin
            w64_tvalid <= 1'b0;
            if (!go_wr && !pack_phase && wword_cnt == 21'd0) begin
                // 只在记录边界停：保证每条 4MB S2MM 命令都能收齐数据
                samp_idx <= {IDX_W{1'b0}};
            end else if (dds_tvalid) begin
                if (pack_phase == 1'b0) begin
                    sample_lo <= samp_in;            // 低样本
                    samp_idx  <= samp_idx + 1'b1;    // 下一样本序号(到 TOTAL_SAMPLES 自然回绕)
                    pack_phase <= 1'b1;
                end else begin
                    w64_tdata  <= {samp_in, sample_lo};   // {高样本, 低样本}
                    samp_idx   <= samp_idx + 1'b1;
                    w64_tvalid <= 1'b1;
                    w64_tlast  <= (wword_cnt == WWORDS-1);
                    wword_cnt  <= (wword_cnt == WWORDS-1) ? 21'd0 : wword_cnt + 21'd1;
                    pack_phase <= 1'b0;
                end
            end
        end
    end

    // 写跨域+位宽：64@clk_wr → 512@ui_clk
    wire [511:0] w512_tdata;
    wire [63:0]  w512_tkeep;
    wire         w512_tvalid, w512_tready, w512_tlast;
    axis_async_fifo_adapter #(
        .DEPTH(4096), .S_DATA_WIDTH(64),  .S_KEEP_ENABLE(1), .S_KEEP_WIDTH(8),
        .M_DATA_WIDTH(512), .M_KEEP_ENABLE(1), .M_KEEP_WIDTH(64),
        .ID_ENABLE(0), .DEST_ENABLE(0), .USER_ENABLE(0), .FRAME_FIFO(0)
    ) u_wfifo (
        .s_clk(clk_wr), .s_rst(rst_wr),
        .s_axis_tdata(w64_tdata), .s_axis_tkeep(8'hff), .s_axis_tvalid(w64_tvalid),
        .s_axis_tready(w64_tready), .s_axis_tlast(w64_tlast),
        .s_axis_tid(0), .s_axis_tdest(0), .s_axis_tuser(0),
        .m_clk(ui_clk), .m_rst(ui_clk_sync_rst),
        .m_axis_tdata(w512_tdata), .m_axis_tkeep(w512_tkeep), .m_axis_tvalid(w512_tvalid),
        .m_axis_tready(w512_tready), .m_axis_tlast(w512_tlast),
        .m_axis_tid(), .m_axis_tdest(), .m_axis_tuser(),
        .s_status_overflow(), .s_status_bad_frame(), .s_status_good_frame(),
        .m_status_overflow(), .m_status_bad_frame(), .m_status_good_frame()
    );

    //=========================================================================
    // 读环 datapath：MM2S 512@ui_clk → 512@clk(250M) → m_axis（不做位宽转换，32 KB FIFO）
    //=========================================================================
    wire [511:0] r512_tdata;
    wire [63:0]  r512_tkeep;
    wire         r512_tvalid, r512_tready, r512_tlast;
    axis_async_fifo_adapter #(
        .DEPTH(32768), .S_DATA_WIDTH(512), .S_KEEP_ENABLE(1), .S_KEEP_WIDTH(64),
        .M_DATA_WIDTH(512), .M_KEEP_ENABLE(1), .M_KEEP_WIDTH(64),
        .ID_ENABLE(0), .DEST_ENABLE(0), .USER_ENABLE(0), .FRAME_FIFO(0)
    ) u_rfifo (
        .s_clk(ui_clk), .s_rst(ui_clk_sync_rst),
        .s_axis_tdata(r512_tdata), .s_axis_tkeep(r512_tkeep), .s_axis_tvalid(r512_tvalid),
        .s_axis_tready(r512_tready), .s_axis_tlast(r512_tlast),
        .s_axis_tid(0), .s_axis_tdest(0), .s_axis_tuser(0),
        .m_clk(clk), .m_rst(rst),
        .m_axis_tdata(m_axis_tdata), .m_axis_tkeep(), .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready), .m_axis_tlast(),
        .m_axis_tid(), .m_axis_tdest(), .m_axis_tuser(),
        .s_status_overflow(), .s_status_bad_frame(), .s_status_good_frame(),
        .m_status_overflow(), .m_status_bad_frame(), .m_status_good_frame()
    );

    //=========================================================================
    // DataMover 命令格式（72bit, BTT=23bit, ADDR=32bit）
    //=========================================================================
    function [71:0] dm_cmd(input [31:0] addr, input [22:0] btt);
        dm_cmd = { 4'd0,    // [71:68] reserved/xuser
                   4'd0,    // [67:64] tag
                   addr,    // [63:32] start addr
                   1'b0,    // [31]    DRR
                   1'b1,    // [30]    EOF
                   6'd0,    // [29:24] DSA
                   1'b1,    // [23]    TYPE = INCR
                   btt };   // [22:0]  BTT
    endfunction

    //=========================================================================
    // 乒乓双缓冲：A 区=BASE_ADDR，B 区=BASE_ADDR+BTT_BYTES
    //   写在 wr_buf 块；写完后把 ready_buf 指向刚写完的块、wr_buf 翻转。
    //   读永远读 ready_buf（已写完、当前没在写的块）→ 整段相干、不撕裂。
    //   (前提：读+发速率 > 写速率，否则读不完会被写追上；100G 下读 98.9 Gbps ≫ 写 4 Gbps)
    //=========================================================================
    reg  wr_buf;        // 当前写哪块 (0=A,1=B)
    reg  ready_buf;     // 最近写完、可读的块
    reg  buf_valid;     // 是否已有至少一块写完
    wire [31:0] wr_addr = wr_buf    ? (BASE_ADDR + BTT_BYTES) : BASE_ADDR;
    wire [31:0] rd_addr = ready_buf ? (BASE_ADDR + BTT_BYTES) : BASE_ADDR;

    //=========================================================================
    // 写命令 FSM（ui_clk）：写满一块→切换，标记可读块
    //=========================================================================
    wire        s2mm_cmd_tready, s2mm_sts_tvalid;
    reg         s2mm_cmd_tvalid;
    reg  [71:0] s2mm_cmd_tdata;
    reg  [1:0]  wst;
    localparam W_ISSUE=2'd0, W_WAIT=2'd1;
    always @(posedge ui_clk) begin
        if (ui_clk_sync_rst) begin
            wst <= W_ISSUE; s2mm_cmd_tvalid <= 1'b0;
            wr_buf <= 1'b0; ready_buf <= 1'b0; buf_valid <= 1'b0;
        end else begin
            case (wst)
            W_ISSUE: begin
                s2mm_cmd_tdata <= dm_cmd(wr_addr, BTT);
                // 写 FIFO 里已有记录(停写时 pack 正在记录中途)也要发命令，否则数据堆在 FIFO
                if ((go_ui && !rd_only) || w512_tvalid)
                    s2mm_cmd_tvalid <= 1'b1;
                if (s2mm_cmd_tvalid && s2mm_cmd_tready) begin
                    s2mm_cmd_tvalid <= 1'b0;
                    wst <= W_WAIT;
                end
            end
            W_WAIT: if (s2mm_sts_tvalid) begin
                ready_buf <= wr_buf;        // 刚写完的块可读
                buf_valid <= 1'b1;
                wr_buf    <= ~wr_buf;        // 下一块写另一边
                wst       <= W_ISSUE;
            end
            endcase
        end
    end

    //=========================================================================
    // 读命令 FSM（ui_clk）：循环读 ready_buf 块（只读模式下直接读 A 区）
    //   最多 2 条命令在途：前一条还没读完就把下一条排进 DataMover，
    //   4MB 命令之间 MM2S 不会断流。
    //=========================================================================
    wire        mm2s_cmd_tready, mm2s_sts_tvalid;
    reg         mm2s_cmd_tvalid;
    reg  [71:0] mm2s_cmd_tdata;
    reg  [1:0]  rd_inflight;
    wire        rd_issue = mm2s_cmd_tvalid && mm2s_cmd_tready;
    always @(posedge ui_clk) begin
        if (ui_clk_sync_rst) begin
            mm2s_cmd_tvalid <= 1'b0; rd_inflight <= 2'd0;
        end else begin
            rd_inflight <= rd_inflight + (rd_issue ? 2'd1 : 2'd0) - (mm2s_sts_tvalid ? 2'd1 : 2'd0);
            if (rd_issue)
                mm2s_cmd_tvalid <= 1'b0;
            else if (!mm2s_cmd_tvalid && go_ui && (buf_valid || rd_only) &&
                     (rd_inflight - (mm2s_sts_tvalid ? 2'd1 : 2'd0)) < 2'd2) begin
                mm2s_cmd_tdata  <= dm_cmd(rd_only ? BASE_ADDR : rd_addr, BTT);
                mm2s_cmd_tvalid <= 1'b1;
            end
        end
    end

    //=========================================================================
    // AXI 线（DataMover MM2S/S2MM <-> MIG S_AXI）
    //=========================================================================
    wire [3:0]  mm2s_arid;   wire [31:0] mm2s_araddr; wire [7:0] mm2s_arlen;
    wire [2:0]  mm2s_arsize; wire [1:0]  mm2s_arburst;wire [2:0] mm2s_arprot;
    wire [3:0]  mm2s_arcache;wire        mm2s_arvalid,mm2s_arready;
    wire [511:0]mm2s_rdata;  wire [1:0]  mm2s_rresp;  wire mm2s_rlast,mm2s_rvalid,mm2s_rready;

    wire [3:0]  s2mm_awid;   wire [31:0] s2mm_awaddr; wire [7:0] s2mm_awlen;
    wire [2:0]  s2mm_awsize; wire [1:0]  s2mm_awburst;wire [2:0] s2mm_awprot;
    wire [3:0]  s2mm_awcache;wire        s2mm_awvalid,s2mm_awready;
    wire [511:0]s2mm_wdata;  wire [63:0] s2mm_wstrb;  wire s2mm_wlast,s2mm_wvalid,s2mm_wready;
    wire [1:0]  s2mm_bresp;  wire        s2mm_bvalid, s2mm_bready;

    //=========================================================================
    // AXI DataMover
    //=========================================================================
    axi_datamover_0 u_dm (
        // MM2S（读 DDR → 流）
        .m_axi_mm2s_aclk           (ui_clk),
        .m_axi_mm2s_aresetn        (aresetn),
        .mm2s_err                  (),
        .m_axis_mm2s_cmdsts_aclk   (ui_clk),
        .m_axis_mm2s_cmdsts_aresetn(aresetn),
        .s_axis_mm2s_cmd_tvalid    (mm2s_cmd_tvalid),
        .s_axis_mm2s_cmd_tready    (mm2s_cmd_tready),
        .s_axis_mm2s_cmd_tdata     (mm2s_cmd_tdata),
        .m_axis_mm2s_sts_tvalid    (mm2s_sts_tvalid),
        .m_axis_mm2s_sts_tready    (1'b1),
        .m_axis_mm2s_sts_tdata     (),
        .m_axis_mm2s_sts_tkeep     (),
        .m_axis_mm2s_sts_tlast     (),
        .m_axi_mm2s_arid           (mm2s_arid),
        .m_axi_mm2s_araddr         (mm2s_araddr),
        .m_axi_mm2s_arlen          (mm2s_arlen),
        .m_axi_mm2s_arsize         (mm2s_arsize),
        .m_axi_mm2s_arburst        (mm2s_arburst),
        .m_axi_mm2s_arprot         (mm2s_arprot),
        .m_axi_mm2s_arcache        (mm2s_arcache),
        .m_axi_mm2s_aruser         (),
        .m_axi_mm2s_arvalid        (mm2s_arvalid),
        .m_axi_mm2s_arready        (mm2s_arready),
        .m_axi_mm2s_rdata          (mm2s_rdata),
        .m_axi_mm2s_rresp          (mm2s_rresp),
        .m_axi_mm2s_rlast          (mm2s_rlast),
        .m_axi_mm2s_rvalid         (mm2s_rvalid),
        .m_axi_mm2s_rready         (mm2s_rready),
        .m_axis_mm2s_tdata         (r512_tdata),
        .m_axis_mm2s_tkeep         (r512_tkeep),
        .m_axis_mm2s_tlast         (r512_tlast),
        .m_axis_mm2s_tvalid        (r512_tvalid),
        .m_axis_mm2s_tready        (r512_tready),
        // S2MM（流 → 写 DDR）
        .m_axi_s2mm_aclk           (ui_clk),
        .m_axi_s2mm_aresetn        (aresetn),
        .s2mm_err                  (),
        .m_axis_s2mm_cmdsts_awclk  (ui_clk),
        .m_axis_s2mm_cmdsts_aresetn(aresetn),
        .s_axis_s2mm_cmd_tvalid    (s2mm_cmd_tvalid),
        .s_axis_s2mm_cmd_tready    (s2mm_cmd_tready),
        .s_axis_s2mm_cmd_tdata     (s2mm_cmd_tdata),
        .m_axis_s2mm_sts_tvalid    (s2mm_sts_tvalid),
        .m_axis_s2mm_sts_tready    (1'b1),
        .m_axis_s2mm_sts_tdata     (),
        .m_axis_s2mm_sts_tkeep     (),
        .m_axis_s2mm_sts_tlast     (),
        .m_axi_s2mm_awid           (s2mm_awid),
        .m_axi_s2mm_awaddr         (s2mm_awaddr),
        .m_axi_s2mm_awlen          (s2mm_awlen),
        .m_axi_s2mm_awsize         (s2mm_awsize),
        .m_axi_s2mm_awburst        (s2mm_awburst),
        .m_axi_s2mm_awprot         (s2mm_awprot),
        .m_axi_s2mm_awcache        (s2mm_awcache),
        .m_axi_s2mm_awuser         (),
        .m_axi_s2mm_awvalid        (s2mm_awvalid),
        .m_axi_s2mm_awready        (s2mm_awready),
        .m_axi_s2mm_wdata          (s2mm_wdata),
        .m_axi_s2mm_wstrb          (s2mm_wstrb),
        .m_axi_s2mm_wlast          (s2mm_wlast),
        .m_axi_s2mm_wvalid         (s2mm_wvalid),
        .m_axi_s2mm_wready         (s2mm_wready),
        .m_axi_s2mm_bresp          (s2mm_bresp),
        .m_axi_s2mm_bvalid         (s2mm_bvalid),
        .m_axi_s2mm_bready         (s2mm_bready),
        .s_axis_s2mm_tdata         (w512_tdata),
        .s_axis_s2mm_tkeep         (w512_tkeep),
        .s_axis_s2mm_tlast         (w512_tlast),
        .s_axis_s2mm_tvalid        (w512_tvalid),
        .s_axis_s2mm_tready        (w512_tready)
    );

    //=========================================================================
    // DDR4 MIG
    //=========================================================================
    ddr4_0 u_ddr4 (
        .c0_init_calib_complete (calib_done),
        .dbg_clk                (),
        .c0_sys_clk_p           (c0_sys_clk_p),
        .c0_sys_clk_n           (c0_sys_clk_n),
        .dbg_bus                (),
        .c0_ddr4_adr            (c0_ddr4_adr),
        .c0_ddr4_ba             (c0_ddr4_ba),
        .c0_ddr4_cke            (c0_ddr4_cke),
        .c0_ddr4_cs_n           (c0_ddr4_cs_n),
        .c0_ddr4_dm_dbi_n       (c0_ddr4_dm_dbi_n),
        .c0_ddr4_dq             (c0_ddr4_dq),
        .c0_ddr4_dqs_c          (c0_ddr4_dqs_c),
        .c0_ddr4_dqs_t          (c0_ddr4_dqs_t),
        .c0_ddr4_odt            (c0_ddr4_odt),
        .c0_ddr4_bg             (c0_ddr4_bg),
        .c0_ddr4_reset_n        (c0_ddr4_reset_n),
        .c0_ddr4_act_n          (c0_ddr4_act_n),
        .c0_ddr4_ck_c           (c0_ddr4_ck_c),
        .c0_ddr4_ck_t           (c0_ddr4_ck_t),
        .c0_ddr4_ui_clk         (ui_clk),
        .c0_ddr4_ui_clk_sync_rst(ui_clk_sync_rst),
        .c0_ddr4_aresetn        (aresetn),
        // 写通道 ← S2MM
        .c0_ddr4_s_axi_awid     (s2mm_awid),
        .c0_ddr4_s_axi_awaddr   (s2mm_awaddr),
        .c0_ddr4_s_axi_awlen    (s2mm_awlen),
        .c0_ddr4_s_axi_awsize   (s2mm_awsize),
        .c0_ddr4_s_axi_awburst  (s2mm_awburst),
        .c0_ddr4_s_axi_awlock   (1'b0),
        .c0_ddr4_s_axi_awcache  (s2mm_awcache),
        .c0_ddr4_s_axi_awprot   (s2mm_awprot),
        .c0_ddr4_s_axi_awqos    (4'd0),
        .c0_ddr4_s_axi_awvalid  (s2mm_awvalid),
        .c0_ddr4_s_axi_awready  (s2mm_awready),
        .c0_ddr4_s_axi_wdata    (s2mm_wdata),
        .c0_ddr4_s_axi_wstrb    (s2mm_wstrb),
        .c0_ddr4_s_axi_wlast    (s2mm_wlast),
        .c0_ddr4_s_axi_wvalid   (s2mm_wvalid),
        .c0_ddr4_s_axi_wready   (s2mm_wready),
        .c0_ddr4_s_axi_bready   (s2mm_bready),
        .c0_ddr4_s_axi_bid      (),
        .c0_ddr4_s_axi_bresp    (s2mm_bresp),
        .c0_ddr4_s_axi_bvalid   (s2mm_bvalid),
        // 读通道 ← MM2S
        .c0_ddr4_s_axi_arid     (mm2s_arid),
        .c0_ddr4_s_axi_araddr   (mm2s_araddr),
        .c0_ddr4_s_axi_arlen    (mm2s_arlen),
        .c0_ddr4_s_axi_arsize   (mm2s_arsize),
        .c0_ddr4_s_axi_arburst  (mm2s_arburst),
        .c0_ddr4_s_axi_arlock   (1'b0),
        .c0_ddr4_s_axi_arcache  (mm2s_arcache),
        .c0_ddr4_s_axi_arprot   (mm2s_arprot),
        .c0_ddr4_s_axi_arqos    (4'd0),
        .c0_ddr4_s_axi_arvalid  (mm2s_arvalid),
        .c0_ddr4_s_axi_arready  (mm2s_arready),
        .c0_ddr4_s_axi_rready   (mm2s_rready),
        .c0_ddr4_s_axi_rlast    (mm2s_rlast),
        .c0_ddr4_s_axi_rvalid   (mm2s_rvalid),
        .c0_ddr4_s_axi_rresp    (mm2s_rresp),
        .c0_ddr4_s_axi_rid      (),
        .c0_ddr4_s_axi_rdata    (mm2s_rdata),
        .sys_rst                (sys_rst)
    );

    //=========================================================================
    // 每秒计数（ui_clk = 300 MHz）：MIG 读 / 写握手拍数
    //=========================================================================
    rate_counter #(.CLK_HZ(300000000)) u_rd_rate (
        .clk(ui_clk), .rst(ui_clk_sync_rst), .inc(mm2s_rvalid & mm2s_rready), .rate(rd_beats_per_s)
    );
    rate_counter #(.CLK_HZ(300000000)) u_wr_rate (
        .clk(ui_clk), .rst(ui_clk_sync_rst), .inc(s2mm_wvalid & s2mm_wready), .rate(wr_beats_per_s)
    );

endmodule
