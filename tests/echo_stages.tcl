# Watch the UDP echo path stage by stage (per-second VIO counters) while a host sends.
#
#   vivado -mode batch -source tests/echo_stages.tcl -tclargs [seconds] [program]
#
# seconds: how long to print (default 30). program = 1 loads build/fpga.bit first.
# Columns (per second): CMAC RX frames, of them flagged bad, RX FIFO drops (full / bad frame),
# frames into the stack, cycles the stack held off the RX FIFO, echo FIFO drops (full),
# echo FIFO packets, CMAC TX frames, CMAC TX Gbit/s.

set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set secs [expr {$argc > 0 ? [lindex $argv 0] : 30}]
set prog [expr {$argc > 1 ? [lindex $argv 1] : 0}]
set bit  [file join $repo_dir build fpga.bit]
set ltx  [file join $repo_dir build fpga.ltx]

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices xczu48dr*] 0]
current_hw_device $dev
set_property PROBES.FILE $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
if {$prog} {
    set_property PROGRAM.FILE $bit $dev
    program_hw_devices $dev
}
refresh_hw_device $dev
set vio [get_hw_vios -of_objects $dev]

set names {mac_rx_frames_per_s mac_rx_bad_per_s rxf_drop_per_s rxf_bad_per_s
           stack_rx_frames_per_s stack_stall_per_s echo_drop_per_s echo_good_per_s
           mac_tx_frames_per_s mac_tx_bytes_per_s}
foreach n $names {
    set p [get_hw_probes core_inst/$n -of_objects $vio]
    set_property INPUT_VALUE_RADIX UNSIGNED $p
    set probe($n) $p
}

puts [format "%4s %9s %6s %9s %6s %9s %11s %9s %9s %9s %7s" \
    t mac_rx rx_bad rxf_drop rxf_bad stack_rx stack_stall echo_drop echo_pkts mac_tx tx_Gbps]
for {set t 1} {$t <= $secs} {incr t} {
    after 1000
    refresh_hw_vio $vio
    foreach n $names { set v($n) [get_property INPUT_VALUE $probe($n)] }
    puts [format "%4d %9s %6s %9s %6s %9s %11s %9s %9s %9s %7.2f" $t \
        $v(mac_rx_frames_per_s) $v(mac_rx_bad_per_s) $v(rxf_drop_per_s) $v(rxf_bad_per_s) \
        $v(stack_rx_frames_per_s) $v(stack_stall_per_s) $v(echo_drop_per_s) $v(echo_good_per_s) \
        $v(mac_tx_frames_per_s) [expr {$v(mac_tx_bytes_per_s) * 8 / 1e9}]]
    flush stdout
}
close_hw_manager
