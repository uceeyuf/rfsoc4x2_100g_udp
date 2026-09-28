#!/bin/bash
# System testbench of udp_stack + record_eth_tx with the Vivado simulator (xsim), same file
# list as sim/run_udp_stack.sh (Icarus Verilog).
#   source <Vivado>/settings64.sh; sim/run_udp_stack_xsim.sh
# Upstream verilog-ethernet files that have a copy in rtl/ or rtl/stack are skipped, and so are
# the PHY / MAC files the testbench does not use.
set -e
cd "$(dirname "$0")/.."
R=$(pwd)
W=build/sim_xsim
rm -rf $W; mkdir -p $W
UP=""
for f in third_party/verilog-ethernet/rtl/*.v third_party/verilog-ethernet/lib/axis/rtl/*.v; do
    b=$(basename $f)
    case $b in ssio_*|iddr*|oddr*|*rgmii*|*gmii*|*mii_phy*|*xgmii*|eth_mac_*|ptp_*|eth_phy_*|axis_baser_*|*_phy_*) continue;; esac
    [ -f rtl/stack/$b ] || [ -f rtl/$b ] || UP="$UP $R/$f"
done
cd $W
xvlog -sv $R/sim/tb_udp_stack.v $R/rtl/udp_stack.v $R/rtl/udp_echo.v $R/rtl/icmp_echo.v $R/rtl/record_eth_tx.v \
    $R/rtl/arp.v $R/rtl/stack/*.v $UP > xvlog.log 2>&1 || { grep ERROR xvlog.log; exit 1; }
xelab -debug off tb_udp_stack -s tb > xelab.log 2>&1 || { grep ERROR xelab.log; exit 1; }
xsim tb -R 2>&1 | grep -E "PASS|ERROR|\*\*\*|TIMEOUT"
