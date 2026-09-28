// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Yijie Yu
//
// RFSoC 4x2 top level: clocking and resets around fpga_core.
//   100 MHz board clock -> MMCM (VCO 1000 MHz) -> 250 MHz clk (core), 125 MHz clk_125

`resetall
`timescale 1ns / 1ps
`default_nettype none

module fpga (
    input  wire         clk_p,              // 100 MHz
    input  wire         clk_n,
    input  wire         reset_n,            // push button, active low

    // QSFP28
    output wire         qsfp0_tx1_p, qsfp0_tx1_n,
    input  wire         qsfp0_rx1_p, qsfp0_rx1_n,
    output wire         qsfp0_tx2_p, qsfp0_tx2_n,
    input  wire         qsfp0_rx2_p, qsfp0_rx2_n,
    output wire         qsfp0_tx3_p, qsfp0_tx3_n,
    input  wire         qsfp0_rx3_p, qsfp0_rx3_n,
    output wire         qsfp0_tx4_p, qsfp0_tx4_n,
    input  wire         qsfp0_rx4_p, qsfp0_rx4_n,
    input  wire         qsfp0_mgt_refclk_0_p,
    input  wire         qsfp0_mgt_refclk_0_n,
    output wire         qsfp0_modsell,
    output wire         qsfp0_resetl,
    input  wire         qsfp0_modprsl,
    input  wire         qsfp0_intl,
    output wire         qsfp0_lpmode,

    // PL DDR4
    input  wire         c0_sys_clk_p,
    input  wire         c0_sys_clk_n,
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
    output wire [0:0]   c0_ddr4_ck_t
);

wire clk_in, clk_fb, mmcm_locked;
wire clk_core_mmcm, clk_125_mmcm;
wire clk_core, clk_125;
wire rst_core, rst_125;

IBUFDS clk_in_ibufds (.I(clk_p), .IB(clk_n), .O(clk_in));

MMCME4_BASE #(
    .CLKIN1_PERIOD(10.0),
    .DIVCLK_DIVIDE(1),
    .CLKFBOUT_MULT_F(10.0),         // VCO = 100 * 10 = 1000 MHz
    .CLKOUT0_DIVIDE_F(4.0),         // 250 MHz
    .CLKOUT1_DIVIDE(8),             // 125 MHz
    .BANDWIDTH("OPTIMIZED"),
    .STARTUP_WAIT("FALSE")
)
clk_mmcm (
    .CLKIN1(clk_in),
    .CLKFBIN(clk_fb),
    .CLKFBOUT(clk_fb),
    .CLKFBOUTB(),
    .RST(~reset_n),
    .PWRDWN(1'b0),
    .CLKOUT0(clk_core_mmcm),
    .CLKOUT0B(),
    .CLKOUT1(clk_125_mmcm),
    .CLKOUT1B(),
    .CLKOUT2(), .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(),
    .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
    .LOCKED(mmcm_locked)
);

BUFG clk_core_bufg (.I(clk_core_mmcm), .O(clk_core));
BUFG clk_125_bufg (.I(clk_125_mmcm), .O(clk_125));

sync_reset #(.N(4)) rst_core_sync (.clk(clk_core), .rst(~mmcm_locked), .out(rst_core));
sync_reset #(.N(4)) rst_125_sync (.clk(clk_125), .rst(~mmcm_locked), .out(rst_125));

fpga_core core_inst (
    .clk(clk_core), .rst(rst_core),
    .clk_125(clk_125), .rst_125(rst_125),
    .qsfp0_tx1_p(qsfp0_tx1_p), .qsfp0_tx1_n(qsfp0_tx1_n), .qsfp0_rx1_p(qsfp0_rx1_p), .qsfp0_rx1_n(qsfp0_rx1_n),
    .qsfp0_tx2_p(qsfp0_tx2_p), .qsfp0_tx2_n(qsfp0_tx2_n), .qsfp0_rx2_p(qsfp0_rx2_p), .qsfp0_rx2_n(qsfp0_rx2_n),
    .qsfp0_tx3_p(qsfp0_tx3_p), .qsfp0_tx3_n(qsfp0_tx3_n), .qsfp0_rx3_p(qsfp0_rx3_p), .qsfp0_rx3_n(qsfp0_rx3_n),
    .qsfp0_tx4_p(qsfp0_tx4_p), .qsfp0_tx4_n(qsfp0_tx4_n), .qsfp0_rx4_p(qsfp0_rx4_p), .qsfp0_rx4_n(qsfp0_rx4_n),
    .qsfp0_mgt_refclk_0_p(qsfp0_mgt_refclk_0_p), .qsfp0_mgt_refclk_0_n(qsfp0_mgt_refclk_0_n),
    .qsfp0_modsell(qsfp0_modsell), .qsfp0_resetl(qsfp0_resetl), .qsfp0_modprsl(qsfp0_modprsl),
    .qsfp0_intl(qsfp0_intl), .qsfp0_lpmode(qsfp0_lpmode),
    .c0_sys_clk_p(c0_sys_clk_p), .c0_sys_clk_n(c0_sys_clk_n), .ddr_sys_rst(~reset_n),
    .c0_ddr4_adr(c0_ddr4_adr), .c0_ddr4_ba(c0_ddr4_ba), .c0_ddr4_cke(c0_ddr4_cke),
    .c0_ddr4_cs_n(c0_ddr4_cs_n), .c0_ddr4_dm_dbi_n(c0_ddr4_dm_dbi_n), .c0_ddr4_dq(c0_ddr4_dq),
    .c0_ddr4_dqs_c(c0_ddr4_dqs_c), .c0_ddr4_dqs_t(c0_ddr4_dqs_t), .c0_ddr4_odt(c0_ddr4_odt),
    .c0_ddr4_bg(c0_ddr4_bg), .c0_ddr4_reset_n(c0_ddr4_reset_n), .c0_ddr4_act_n(c0_ddr4_act_n),
    .c0_ddr4_ck_c(c0_ddr4_ck_c), .c0_ddr4_ck_t(c0_ddr4_ck_t)
);

endmodule

`resetall
