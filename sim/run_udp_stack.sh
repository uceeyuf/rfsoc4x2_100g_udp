#!/bin/sh
# System testbench of udp_stack + record_eth_tx with Icarus Verilog.
#   sim/run_udp_stack.sh
# Upstream verilog-ethernet files that have a copy in rtl/ or rtl/stack are skipped.
set -e
cd "$(dirname "$0")/.."
OUT=build/sim_tb
mkdir -p "$OUT"
UP=""
for f in third_party/verilog-ethernet/rtl/*.v third_party/verilog-ethernet/lib/axis/rtl/*.v; do
    b=$(basename "$f")
    [ -f "rtl/stack/$b" ] || [ -f "rtl/$b" ] || UP="$UP $f"
done
rm -f "$OUT/tb_udp_stack.vvp"
iverilog -g2012 -s tb_udp_stack -o "$OUT/tb_udp_stack.vvp" sim/tb_udp_stack.v \
    rtl/udp_stack.v rtl/udp_echo.v rtl/icmp_echo.v rtl/record_eth_tx.v rtl/arp.v rtl/stack/*.v $UP \
    2> "$OUT/iverilog.log" || { grep -i error "$OUT/iverilog.log"; exit 1; }
vvp -n "$OUT/tb_udp_stack.vvp" | grep -E "PASS|ERROR|\*\*\*"
