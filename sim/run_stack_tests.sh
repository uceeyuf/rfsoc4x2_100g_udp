#!/bin/sh
# Run the rtl/stack testbenches with Icarus Verilog at a given datapath width.
#   sim/run_stack_tests.sh [width=512]
# The testbenches declare "localparam DW/DATA_WIDTH = 512"; a patched copy is made per width.
set -e
W=${1:-512}
cd "$(dirname "$0")/.."
OUT=build/sim_stack_$W
mkdir -p "$OUT"
# upstream files that have a modified copy in rtl/stack are skipped
UP=""
for f in third_party/verilog-ethernet/rtl/*.v third_party/verilog-ethernet/lib/axis/rtl/*.v; do
    [ -f "rtl/stack/$(basename "$f")" ] || UP="$UP $f"
done
SRC="rtl/stack/*.v$UP"
for f in sim/stack/tb_*.v; do
    tb=$(basename "$f" .v)
    sed -E "s/(localparam +DW *= *)512/\1$W/; s/(localparam +DATA_WIDTH *= *)512/\1$W/" "$f" > "$OUT/$tb.v"
    echo "=== $tb @ $W"
    iverilog -g2012 -s "$tb" -o "$OUT/$tb.vvp" "$OUT/$tb.v" $SRC 2> "$OUT/$tb.err" || { grep -vi warning "$OUT/$tb.err" | head -5; continue; }
    vvp -n "$OUT/$tb.vvp" | grep -E "PASS|FAIL|\*\*\*" | tail -5
done
