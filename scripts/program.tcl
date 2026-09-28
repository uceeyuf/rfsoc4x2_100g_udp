# Program the RFSoC 4x2 over JTAG with build/fpga.bit (+ fpga.ltx for the VIO).
#
#   vivado -mode batch -source scripts/program.tcl [-tclargs <bit> [<ltx>]]

set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set bit [expr {$argc > 0 ? [lindex $argv 0] : [file join $repo_dir build fpga.bit]}]
set ltx [expr {$argc > 1 ? [lindex $argv 1] : [file join $repo_dir build fpga.ltx]}]

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

set dev [lindex [get_hw_devices xczu48dr*] 0]
current_hw_device $dev
set_property PROGRAM.FILE $bit $dev
if {[file exists $ltx]} {
    set_property PROBES.FILE      $ltx $dev
    set_property FULL_PROBES.FILE $ltx $dev
}
program_hw_devices $dev
refresh_hw_device $dev
puts "Programmed $dev with $bit"
