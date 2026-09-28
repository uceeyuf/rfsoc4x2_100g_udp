# Stop / start the stream several times and check that every packet carries the data its
# header says (ramp test data, see tests/restart_check.py).
#
#   vivado -mode batch -source tests/restart_check.tcl -tclargs [cycles=5] [header_bytes=22] [packets=3000]
#
# One flow, standard packets, slowed down with tx_delay so that Python keeps up.

set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set cycles [expr {$argc > 0 ? [lindex $argv 0] : 5}]
set hdr    [expr {$argc > 1 ? [lindex $argv 1] : 22}]
set npkt   [expr {$argc > 2 ? [lindex $argv 2] : 3000}]
set python python3
set ltx [expr {[info exists ::env(LTX)] ? $::env(LTX) : [file join $repo_dir build fpga.ltx]}]

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices xczu48dr*] 0]
current_hw_device $dev
set_property PROBES.FILE $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
refresh_hw_device $dev
set vio [get_hw_vios -of_objects $dev]

proc set_out {name val} {
    set p [get_hw_probes core_inst/$name -of_objects $::vio]
    set_property OUTPUT_VALUE_RADIX UNSIGNED $p
    set_property OUTPUT_VALUE $val $p
    commit_hw_vio $p
}

set_out tx_speed_en 0
set_out tx_length 1
set_out tx_delay 20000
set fails 0
for {set c 1} {$c <= $cycles} {incr c} {
    set py [open "|[list $python [file join $repo_dir tests restart_check.py] $hdr $npkt]" r]
    gets $py line                           ;# READY
    after 300
    set_out tx_speed_en 1
    set out [read $py]
    catch {close $py}
    set_out tx_speed_en 0
    after 1000
    regexp {RESULT.*} $out res
    puts "cycle $c: $res"
    if {![regexp {bad 0} $res] || ![regexp {first_start_index 0 } $res]} { incr fails }
}
set_out tx_delay 0
set_out tx_length 0
puts [expr {$fails ? "RESTART CHECK: $fails of $cycles cycles FAILED" : "RESTART CHECK: all $cycles cycles OK"}]
