# Stream control and benchmark counters.
#   in0 txstate[3:0], in1 MIG read beats/s, in2 MIG write beats/s, in3 beats taken at the core clock /s,
#   in4 cycles waiting for DDR data /s, in5 bytes to the CMAC /s (40 bit)
#   echo path /s: in6 CMAC RX frames, in7 of them flagged bad, in8 RX FIFO drops (full),
#   in9 RX FIFO drops (bad frame), in10 frames into the stack, in11 stack back-pressure cycles,
#   in12 echo FIFO drops (full), in13 echo FIFO packets, in14 CMAC TX frames
#   out0 tx_speed_en, out1 tx_length[15:0], out2 tx_delay[15:0], out3 bench_ctrl[7:0]
create_ip -name vio -vendor xilinx.com -library ip -module_name vio_0
set_property -dict [list \
    CONFIG.C_NUM_PROBE_IN {15} \
    CONFIG.C_PROBE_IN0_WIDTH {4} \
    CONFIG.C_PROBE_IN1_WIDTH {32} \
    CONFIG.C_PROBE_IN2_WIDTH {32} \
    CONFIG.C_PROBE_IN3_WIDTH {32} \
    CONFIG.C_PROBE_IN4_WIDTH {32} \
    CONFIG.C_PROBE_IN5_WIDTH {40} \
    CONFIG.C_PROBE_IN6_WIDTH {32} \
    CONFIG.C_PROBE_IN7_WIDTH {32} \
    CONFIG.C_PROBE_IN8_WIDTH {32} \
    CONFIG.C_PROBE_IN9_WIDTH {32} \
    CONFIG.C_PROBE_IN10_WIDTH {32} \
    CONFIG.C_PROBE_IN11_WIDTH {32} \
    CONFIG.C_PROBE_IN12_WIDTH {32} \
    CONFIG.C_PROBE_IN13_WIDTH {32} \
    CONFIG.C_PROBE_IN14_WIDTH {32} \
    CONFIG.C_NUM_PROBE_OUT {4} \
    CONFIG.C_PROBE_OUT0_WIDTH {1} \
    CONFIG.C_PROBE_OUT1_WIDTH {16} \
    CONFIG.C_PROBE_OUT2_WIDTH {16} \
    CONFIG.C_PROBE_OUT3_WIDTH {8} \
    CONFIG.C_PROBE_OUT1_INIT_VAL {0x0000} \
    CONFIG.C_PROBE_OUT2_INIT_VAL {0x0000} \
    CONFIG.C_PROBE_OUT3_INIT_VAL {0x00} \
    CONFIG.C_EN_PROBE_IN_ACTIVITY {0} \
] [get_ips vio_0]
