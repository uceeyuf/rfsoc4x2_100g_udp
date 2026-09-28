`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Testbench: tb_ddr_record_loop
//   仿真 ddr_record_loop（DDS + DDR4 MIG(BFM) + AXI DataMover 持续写/读环）
//
//   前提：ddr4_0 的 Simulation Mode = BFM（你已设）。BFM 下 MIG 校准秒过、内部
//         行为模型代替 DDR 颗粒，c0_ddr4_* 物理口在仿真里悬空即可。
//
//   为加快仿真，TOTAL_SAMPLES 压到 4096（4MB 实样要 8ms 仿真太久）。
//
//   看点：
//     1) init_calib_complete 拉高（DDR 就绪）
//     2) m_axis_tvalid 出数据、m_axis_tdata 随时间变化（正弦，非常量/非零卡死）
//     3) 低 16bit=cos、高 16bit=sin 大致是反相/正交的正弦
//   运行：把本文件设为 sim 顶层，xsim 跑 ~300us。
//////////////////////////////////////////////////////////////////////////////////

module tb_ddr_record_loop;

    // 125MHz 以太/DDS 时钟（周期 8ns）
    reg clk = 1'b0;
    always #4.0 clk = ~clk;

    // 200MHz DDR 参考时钟（周期 5ns）
    reg sysclk = 1'b0;
    always #2.5 sysclk = ~sysclk;
    wire c0_sys_clk_p =  sysclk;
    wire c0_sys_clk_n = ~sysclk;

    reg rst     = 1'b1;
    reg sys_rst = 1'b1;     // MIG 高有效复位
    reg trig    = 1'b0;

    // record_eth_tx 一侧的消费者：用带间隙的 tready 模拟反压
    reg  m_axis_tready = 1'b1;
    wire [63:0] m_axis_tdata;
    wire        m_axis_tvalid;
    wire        init_calib_complete;

    // DDR 物理口（BFM 模式下悬空）
    wire [16:0] c0_ddr4_adr;
    wire [1:0]  c0_ddr4_ba;
    wire [0:0]  c0_ddr4_cke;
    wire [0:0]  c0_ddr4_cs_n;
    wire [7:0]  c0_ddr4_dm_dbi_n;
    wire [63:0] c0_ddr4_dq;
    wire [7:0]  c0_ddr4_dqs_c;
    wire [7:0]  c0_ddr4_dqs_t;
    wire [0:0]  c0_ddr4_odt;
    wire [0:0]  c0_ddr4_bg;
    wire        c0_ddr4_reset_n;
    wire        c0_ddr4_act_n;
    wire [0:0]  c0_ddr4_ck_c;
    wire [0:0]  c0_ddr4_ck_t;

    // DUT（小样本加速仿真）
    ddr_record_loop #(
        .TOTAL_SAMPLES(4096)
    ) dut (
        .clk(clk), .rst(rst), .trig(trig),
        .clk_wr(clk), .rst_wr(rst),          // 仿真里写侧与读侧同钟
        .c0_sys_clk_p(c0_sys_clk_p), .c0_sys_clk_n(c0_sys_clk_n),
        .sys_rst(sys_rst), .init_calib_complete(init_calib_complete),
        .c0_ddr4_adr(c0_ddr4_adr), .c0_ddr4_ba(c0_ddr4_ba), .c0_ddr4_cke(c0_ddr4_cke),
        .c0_ddr4_cs_n(c0_ddr4_cs_n), .c0_ddr4_dm_dbi_n(c0_ddr4_dm_dbi_n), .c0_ddr4_dq(c0_ddr4_dq),
        .c0_ddr4_dqs_c(c0_ddr4_dqs_c), .c0_ddr4_dqs_t(c0_ddr4_dqs_t), .c0_ddr4_odt(c0_ddr4_odt),
        .c0_ddr4_bg(c0_ddr4_bg), .c0_ddr4_reset_n(c0_ddr4_reset_n), .c0_ddr4_act_n(c0_ddr4_act_n),
        .c0_ddr4_ck_c(c0_ddr4_ck_c), .c0_ddr4_ck_t(c0_ddr4_ck_t),
        .m_axis_tdata(m_axis_tdata), .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready)
    );

    // 复位/触发时序
    initial begin
        rst     = 1'b1;
        sys_rst = 1'b1;
        trig    = 1'b0;
        #200;
        sys_rst = 1'b0;          // 释放 MIG 复位
        #200;
        rst     = 1'b0;          // 释放逻辑复位
        // 等校准完成再触发
        wait (init_calib_complete === 1'b1);
        $display("[%0t] init_calib_complete=1，DDR 就绪", $time);
        #1000;
        trig = 1'b1;             // 开始持续写+读
        $display("[%0t] trig=1，开始写/读环", $time);
    end

    // tready 加点间隙模拟反压（每 8 拍停 1 拍）
    integer rcnt = 0;
    always @(posedge clk) begin
        rcnt <= rcnt + 1;
        m_axis_tready <= (rcnt % 8 != 0);
    end

    // 监测 m_axis 输出，打印前若干个样本（拆 cos/sin）
    integer beats = 0;
    integer fd;
    initial fd = $fopen("ddr_loop_out.txt", "w");
    always @(posedge clk) begin
        if (m_axis_tvalid && m_axis_tready) begin
            beats = beats + 1;
            // 64bit = {sample1, sample0}; sample={sin[15:0],cos[15:0]}
            $fwrite(fd, "%0d %0d %0d %0d %0d\n", beats,
                    $signed(m_axis_tdata[15:0]),   $signed(m_axis_tdata[31:16]),
                    $signed(m_axis_tdata[47:32]),  $signed(m_axis_tdata[63:48]));
            if (beats <= 20)
                $display("[%0t] beat=%0d cos0=%0d sin0=%0d cos1=%0d sin1=%0d", $time, beats,
                    $signed(m_axis_tdata[15:0]),  $signed(m_axis_tdata[31:16]),
                    $signed(m_axis_tdata[47:32]), $signed(m_axis_tdata[63:48]));
        end
    end

    // 校准超时告警
    initial begin
        #500000;
        if (init_calib_complete !== 1'b1)
            $display("[%0t] *** 警告：500us 仍未 calib，检查 BFM/时钟/复位 ***", $time);
    end

    // 收够样本或超时收尾
    initial begin
        #2000000;     // 2ms 仿真上限
        $display("[%0t] 结束：共收到 %0d 个 m_axis beat", $time, beats);
        if (beats > 100) $display(">>> 数据通路 OK：m_axis 持续出数 <<<");
        else             $display(">>> 注意：m_axis 出数很少，查 calib/命令握手/跨时钟 <<<");
        $fclose(fd);
        $finish;
    end

endmodule
