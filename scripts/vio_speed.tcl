# Drive the speed-test VIO (core_inst/vio_0) from the command line.
#
#   vivado -mode batch -source scripts/vio_speed.tcl -tclargs <0|1> [tx_delay] [tx_length]
#
# 1 = start the UDP stream (FPGA:1236 -> PC:1237), 0 = stop.
# tx_delay  = idle clk cycles between packets (0 = full rate).
# tx_length = bits[4:0] number of UDP flows (ports +i), bit[5] also rotate the source IP, bit[11] jumbo;
#             e.g. 40 = 8 flows, 48 = 16 flows with port+IP rotation (spreads over the RSS queues of the PC).
# The PC must have sent at least one UDP packet to the FPGA first,
# so the FPGA has latched the destination MAC/IP (host/dpdk_stream_rx does this).

set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set en    [expr {$argc > 0 ? [lindex $argv 0] : 1}]
set delay [expr {$argc > 1 ? [lindex $argv 1] : ""}]
set len   [expr {$argc > 2 ? [lindex $argv 2] : ""}]
set ltx [file join $repo_dir build fpga.ltx]

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

set dev [lindex [get_hw_devices xczu48dr*] 0]
current_hw_device $dev
set_property PROBES.FILE      $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
refresh_hw_device $dev

set vio [get_hw_vios -of_objects $dev]
foreach {name val} [list tx_delay $delay tx_length $len] {
    if {$val eq ""} continue
    set p [get_hw_probes core_inst/$name -of_objects $vio]
    set_property OUTPUT_VALUE_RADIX UNSIGNED $p
    set_property OUTPUT_VALUE $val $p
    commit_hw_vio $p
}
set_property OUTPUT_VALUE $en [get_hw_probes core_inst/tx_speed_en -of_objects $vio]
commit_hw_vio [get_hw_probes core_inst/tx_speed_en -of_objects $vio]
refresh_hw_vio $vio

foreach p [get_hw_probes -of_objects $vio] {
    set dir [get_property TYPE $p]
    if {$dir eq "vio_output"} {
        puts [format "%-28s OUTPUT = %s" $p [get_property OUTPUT_VALUE $p]]
    } else {
        puts [format "%-28s INPUT  = %s" $p [get_property INPUT_VALUE $p]]
    }
}
