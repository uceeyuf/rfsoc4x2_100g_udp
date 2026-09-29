# DDR -> 100G benchmark, read from the per-second counters in the FPGA (VIO).
#
#   vivado -mode batch -source tests/bench.tcl [-tclargs <seconds per step> [steps...]]
#
# Steps:
#   1  DDR read only        -> discarded at the core clock   : MIG read rate
#   2  DDR read + DDR write -> sink                           : read rate with the 4 Gbps writes
#   3  DDR read only        -> UDP -> CMAC (8 flows, jumbo)   : end-to-end TX rate
#   4  DDR read + write     -> UDP -> CMAC                    : end-to-end with writes
# Steps 3/4 stream to whoever sent the FPGA the last UDP packet: run host/dpdk_stream_rx
# (it registers itself) to receive and check the packets at the same time.

set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set secs  [expr {$argc > 0 ? [lindex $argv 0] : 4}]
set steps [expr {$argc > 1 ? [lrange $argv 1 end] : {1 2 3 4}}]
set core_hz 250e6

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices xczu48dr*] 0]
current_hw_device $dev
set ltx [file join $repo_dir build fpga.ltx]
set_property PROBES.FILE $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
refresh_hw_device $dev
set vio [get_hw_vios -of_objects $dev]

proc probe {name} { return [get_hw_probes core_inst/$name -of_objects $::vio] }
proc set_out {name val} {
    set p [probe $name]
    set_property OUTPUT_VALUE_RADIX UNSIGNED $p
    set_property OUTPUT_VALUE $val $p
    commit_hw_vio $p
}
proc get_in {name} {
    set p [probe $name]
    set_property INPUT_VALUE_RADIX UNSIGNED $p
    refresh_hw_vio $::vio
    return [get_property INPUT_VALUE $p]
}

set rows {}
foreach {step desc ctrl len} {
    1 "DDR read -> sink"             1 0
    2 "DDR read + write -> sink"     0 0
    3 "DDR read -> UDP -> CMAC"      2 2088
    4 "DDR read + write -> UDP"      0 2088
} {
    if {$step ni $steps} continue
    # bench_ctrl: bit0 sink, bit1 no DDR write
    set bench [expr {$step == 1 ? 3 : $step == 2 ? 1 : $step == 3 ? 2 : 0}]
    set_out tx_speed_en 0
    after 500
    set_out bench_ctrl $bench
    set_out tx_length $len
    set_out tx_delay 0
    set_out tx_speed_en 1
    after [expr {int($secs * 1000)}]
    set rd  [get_in rd_beats_per_s]
    set wr  [get_in wr_beats_per_s]
    set out [get_in out_beats_per_s]
    set stv [get_in starve_per_s]
    set mac [get_in mac_tx_bytes_per_s]
    set r [list $step $desc \
        [format %.2f [expr {$rd * 512.0 / 1e9}]] \
        [format %.2f [expr {$wr * 512.0 / 1e9}]] \
        [format %.2f [expr {$out * 512.0 / 1e9}]] \
        [format %.2f [expr {100.0 * $stv / $core_hz}]] \
        [format %.2f [expr {$mac * 8.0 / 1e9}]]]
    lappend rows $r
    puts [format "step %s %-28s MIG rd %7s  MIG wr %6s  core out %7s Gbps  waiting %6s %%  CMAC %7s Gbps" {*}$r]
}
set_out tx_speed_en 0
set_out bench_ctrl 0
set_out tx_length 0

puts "\n==================== DDR4-2400 -> 100G benchmark (${secs} s per step) ===================="
puts [format "%-4s %-28s %10s %10s %12s %10s %12s" step mode "MIG rd" "MIG wr" "core out" "wait %" "CMAC"]
puts [format "%-4s %-28s %10s %10s %12s %10s %12s" "" "" "(Gbps)" "(Gbps)" "(Gbps)" "" "(Gbps)"]
foreach r $rows { puts [format "%-4s %-28s %10s %10s %12s %10s %12s" {*}$r] }
set f [open [file join $repo_dir build bench.csv] w]
puts $f "step,mode,mig_rd_gbps,mig_wr_gbps,out_gbps,wait_pct,cmac_gbps"
foreach r $rows { puts $f [join $r ,] }
close $f
