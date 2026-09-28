# Recreate the Vivado project from the sources in this repository.
#
#   vivado -mode batch -source scripts/create_project.tcl
#
# The project is created in build/rfsoc4x2_100g_udp (ignored by git).
# Tested with Vivado 2023.2.

set repo_dir  [file normalize [file join [file dirname [info script]] ..]]
set proj_name rfsoc4x2_100g_udp
set proj_dir  [file join $repo_dir build $proj_name]
set part      xczu48dr-ffvg1517-2-e

create_project $proj_name $proj_dir -part $part -force

# Board files are optional; only the part is needed to build.
set board [lindex [get_board_parts -quiet -latest_file_version {realdigital.org:rfsoc4x2:*}] 0]
if {$board ne ""} {
    set_property board_part $board [current_project]
}

set_property target_language Verilog [current_project]
set_property default_lib xil_defaultlib [current_project]

# ---------------------------------------------------------------- sources
# rtl/ holds our modules, rtl/stack/ the 512-bit stack (width-parameterized rewrites and
# modified copies of verilog-ethernet files); the rest comes from the submodule.
set own_files [concat [glob [file join $repo_dir rtl *.v]] [glob [file join $repo_dir rtl stack *.v]]]
set own_names {}
foreach f $own_files { lappend own_names [file tail $f] }
set upstream_files {}
foreach f [concat \
        [glob [file join $repo_dir third_party verilog-ethernet rtl *.v]] \
        [glob [file join $repo_dir third_party verilog-ethernet lib axis rtl *.v]]] {
    if {[file tail $f] ni $own_names} { lappend upstream_files $f }
}
add_files -norecurse -fileset sources_1 [concat $own_files $upstream_files]

# IP: import (copy) the .xci so generated products stay inside build/,
# and create the IPs that are described by Tcl (ip/*.tcl)
foreach xci [glob [file join $repo_dir ip * *.xci]] {
    import_ip $xci
}
foreach ip_tcl [glob [file join $repo_dir ip *.tcl]] {
    source $ip_tcl
}
upgrade_ip -quiet [get_ips]

set_property top fpga [get_filesets sources_1]

# ---------------------------------------------------------------- constraints
add_files -norecurse -fileset constrs_1 [list \
    [file join $repo_dir constraints fpga.xdc] \
    [file join $repo_dir constraints 4x2_PL_DDR4.xdc] \
    [file join $repo_dir constraints cdc.xdc]]
set_property target_constrs_file [file join $repo_dir constraints fpga.xdc] [get_filesets constrs_1]

# Upstream per-instance CDC constraints for axis_async_fifo / sync_reset (implementation only,
# applied late so the netlist cell names they look up exist).
foreach tcl {axis_async_fifo.tcl sync_reset.tcl} {
    set f [file join $repo_dir third_party verilog-ethernet lib axis syn vivado $tcl]
    add_files -norecurse -fileset constrs_1 $f
    set obj [get_files $f]
    set_property file_type TCL $obj
    set_property used_in_synthesis false $obj
    set_property used_in_implementation true $obj
    set_property processing_order LATE $obj
}

# ---------------------------------------------------------------- simulation
add_files -norecurse -fileset sim_1 [file join $repo_dir sim tb_ddr_record_loop.v]
set_property top tb_ddr_record_loop [get_filesets sim_1]

# ---------------------------------------------------------------- runs
# 512-bit datapath at 250 MHz; use the timing-driven implementation strategy.
set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]

update_compile_order -fileset sources_1
generate_target all [get_ips]

puts "Project created: [file join $proj_dir $proj_name.xpr]"
