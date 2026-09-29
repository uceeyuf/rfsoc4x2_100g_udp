# Synthesize, implement and write the bitstream.
#
#   vivado -mode batch -source scripts/build.tcl [-tclargs <jobs>]
#
# Creates the project first if it does not exist. Output: build/fpga.bit, build/fpga.ltx

set repo_dir  [file normalize [file join [file dirname [info script]] ..]]
set proj_name rfsoc4x2_100g_udp
set proj_file [file join $repo_dir build $proj_name $proj_name.xpr]
set jobs      [expr {$argc > 0 ? [lindex $argv 0] : 8}]

if {![file exists $proj_file]} {
    source [file join $repo_dir scripts create_project.tcl]
} else {
    open_project $proj_file
}

set synth [get_runs synth_1]
if {[get_property PROGRESS $synth] ne "100%" || [get_property NEEDS_REFRESH $synth]} {
    reset_run synth_1
    launch_runs synth_1 -jobs $jobs
    wait_on_run synth_1
    if {[get_property PROGRESS $synth] ne "100%"} {
        error "synth_1 failed"
    }
}

# On Windows the MIG PHY re-synthesis inside opt_design occasionally fails with
# "couldn't read file .../unimacro_verilog.tcl" (tool file access race), so retry.
for {set attempt 1} {$attempt <= 5} {incr attempt} {
    reset_run impl_1
    launch_runs impl_1 -to_step write_bitstream -jobs $jobs
    if {[catch {wait_on_run impl_1}]} {}
    if {[get_property PROGRESS [get_runs impl_1]] eq "100%"} break
    puts "impl_1 attempt $attempt failed"
}
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    error "impl_1 failed"
}

open_run impl_1
report_timing_summary -file [file join $repo_dir build timing_summary.rpt]
report_utilization    -file [file join $repo_dir build utilization.rpt]

set impl_dir [get_property DIRECTORY [get_runs impl_1]]
file copy -force [file join $impl_dir fpga.bit] [file join $repo_dir build fpga.bit]
if {[file exists [file join $impl_dir fpga.ltx]]} {
    file copy -force [file join $impl_dir fpga.ltx] [file join $repo_dir build fpga.ltx]
}
puts "Bitstream: [file join $repo_dir build fpga.bit]"
