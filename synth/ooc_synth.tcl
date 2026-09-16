# =============================================================================
# ooc_synth.tcl -- Vivado non-project, out-of-context build of ptp_tsu_top
#
#   vivado -mode batch -source ooc_synth.tcl                       (defaults)
#   vivado -mode batch -source ooc_synth.tcl -tclargs <part> <synth|impl>
#
#   part : default xc7z020clg400-1 (PYNQ-Z2).  Any part the installed
#          edition supports works, e.g. xc7a35tcpg236-1 (Artix-7).
#   flow : synth = synthesis only
#          impl  = synthesis + opt / place / phys_opt / route   (default)
#
# Outputs
#   synth/reports/<part>/            committed reports
#     summary.txt                    Fmax, WNS/WHS, LUT/FF/BRAM, CDC checks
#     post_synth_*.rpt, post_route_*.rpt
#   synth/vivado/<part>/             checkpoints (git-ignored)
#
# Run from a path WITHOUT spaces (e.g. C:/work/ptp-timestamping-unit):
# Vivado's Tcl handling of paths with spaces is unreliable.
# =============================================================================

set part [expr {[llength $argv] > 0 ? [lindex $argv 0] : "xc7z020clg400-1"}]
set flow [expr {[llength $argv] > 1 ? [lindex $argv 1] : "impl"}]
set top  ptp_tsu_top

set here [file normalize [file dirname [info script]]]
set root [file normalize [file join $here ..]]
set rpt  [file join $here reports $part]
set chk  [file join $here vivado $part]
file mkdir $rpt $chk

if {[string match "* *" $root]} {
    puts "WARNING: repo path contains spaces: $root"
    puts "WARNING: copy the repo to a path without spaces if Vivado errors on file access."
}

puts "=== ptp_tsu_top OOC build: part $part, flow $flow ==="
set t_start [clock seconds]

# ---- sources ------------------------------------------------------------------
create_project -in_memory -part $part
set_property target_language Verilog [current_project]

set srcs [lsort [glob -directory [file join $root rtl] *.sv]]
foreach f $srcs { read_verilog -sv $f }
puts "read [llength $srcs] RTL files"
read_xdc [file join $here ptp_tsu_clocks.xdc]

# ---- synthesis ----------------------------------------------------------------
synth_design -top $top -part $part -mode out_of_context -flatten_hierarchy rebuilt

# ---- CDC constraints: verify every object exists, then apply -------------------
set cdc_objs {
    u_snap/req_tgl_reg   {u_snap/req_sync_reg[0]}
    u_snap/ack_tgl_reg   {u_snap/ack_sync_reg[0]}
    {u_snap/snap_reg[*]} {u_snap/dst_sec_reg[*]} {u_snap/dst_ns_reg[*]}
    u_bridge/req_tgl_reg {u_bridge/req_sync_reg[0]}
    u_bridge/ack_tgl_reg {u_bridge/ack_sync_reg[0]}
    {u_bridge/addr_hold_reg[*]} {u_bridge/wdata_hold_reg[*]} u_bridge/we_hold_reg
    {u_bridge/s_addr_reg[*]}    {u_bridge/s_wdata_reg[*]}    u_bridge/s_we_reg
    {u_bridge/rdata_hold_reg[*]} {u_bridge/m_rdata_reg[*]}
}
set cdc_missing {}
foreach o $cdc_objs {
    if {[llength [get_cells -quiet $o]] == 0} { lappend cdc_missing $o }
}
if {[llength $cdc_missing] > 0} {
    puts "CRITICAL WARNING: CDC constraint objects not found after synthesis:"
    foreach o $cdc_missing { puts "    $o" }
}
source [file join $here ptp_cdc.xdc]

# ---- helpers --------------------------------------------------------------------
proc clock_summary {clk} {
    set period [get_property PERIOD [get_clocks $clk]]
    set setup  [get_timing_paths -quiet -delay_type max -max_paths 1 \
                    -from [get_clocks $clk] -to [get_clocks $clk]]
    set hold   [get_timing_paths -quiet -delay_type min -max_paths 1 \
                    -from [get_clocks $clk] -to [get_clocks $clk]]
    if {[llength $setup] == 0} {
        return [format "%-8s period %6.3f ns  (no intra-clock paths)" $clk $period]
    }
    set wns  [get_property SLACK $setup]
    set whs  [expr {[llength $hold] ? [get_property SLACK $hold] : "NA"}]
    set fmax [expr {1000.0 / ($period - $wns)}]
    set lvls "?"
    catch { set lvls [get_property LOGIC_LEVELS $setup] }
    set s [format "%-8s period %6.3f ns (%7.2f MHz)  WNS %+7.3f ns  WHS %s ns  Fmax %7.2f MHz" \
               $clk $period [expr {1000.0 / $period}] $wns $whs $fmax]
    append s [format "\n         critical path: %d logic levels, %s -> %s" \
               $lvls [get_property STARTPOINT_PIN $setup] [get_property ENDPOINT_PIN $setup]]
    return $s
}

proc cdc_summary {from to} {
    set p [get_timing_paths -quiet -delay_type max -max_paths 1 \
               -from [get_clocks $from] -to [get_clocks $to]]
    if {[llength $p] == 0} { return [format "%-8s -> %-8s no timed paths" $from $to] }
    return [format "%-8s -> %-8s worst constrained CDC slack %+7.3f ns" \
                $from $to [get_property SLACK $p]]
}

proc util_field {txt label} {
    # matches e.g. "| Slice LUTs*  |  1234 |" and "| Block RAM Tile | 0.5 |"
    if {[regexp "\\|\\s*${label}\\*?\\s*\\|\\s*(\[0-9.\]+)" $txt -> v]} { return $v }
    return "?"
}

proc write_reports {stage rpt} {
    report_timing_summary -max_paths 20 -report_unconstrained -file [file join $rpt ${stage}_timing_summary.rpt]
    report_timing -delay_type max -max_paths 10 -sort_by group -file [file join $rpt ${stage}_timing_paths.rpt]
    report_utilization -file [file join $rpt ${stage}_utilization.rpt]
    report_utilization -hierarchical -hierarchical_depth 2 -file [file join $rpt ${stage}_utilization_hier.rpt]
    report_cdc -details -file [file join $rpt ${stage}_cdc.rpt]
    catch { report_bus_skew -file [file join $rpt ${stage}_bus_skew.rpt] }
    catch { report_exceptions -ignored -file [file join $rpt ${stage}_exceptions_ignored.rpt] }
    catch { report_methodology -file [file join $rpt ${stage}_methodology.rpt] }
    check_timing -verbose -file [file join $rpt ${stage}_check_timing.rpt]

    set u [report_utilization -return_string]
    set s "--- $stage ---\n"
    append s [clock_summary net_clk] "\n"
    append s [clock_summary aclk] "\n"
    append s [cdc_summary net_clk aclk] "\n"
    append s [cdc_summary aclk net_clk] "\n"
    append s [format "LUTs %s (logic %s, memory %s)   FFs %s   BRAM tiles %s   DSPs %s\n" \
                 [util_field $u "Slice LUTs"] [util_field $u "LUT as Logic"] \
                 [util_field $u "LUT as Memory"] [util_field $u "Slice Registers"] \
                 [util_field $u "Block RAM Tile"] [util_field $u "DSPs"]]
    set cdc_txt ""
    catch { set fh [open [file join $rpt ${stage}_cdc.rpt] r]; set cdc_txt [read $fh]; close $fh }
    append s [format "report_cdc: %d Critical, %d Warning entries (see ${stage}_cdc.rpt)\n" \
                 [regexp -all {\mCritical\M} $cdc_txt] [regexp -all {\mWarning\M} $cdc_txt]]
    return $s
}

# ---- post-synthesis ----------------------------------------------------------------
set summary [format "ptp_tsu_top out-of-context build\npart %s   Vivado %s   %s\n\n" \
                 $part [version -short] [clock format [clock seconds] -format "%Y-%m-%d %H:%M"]]
if {[llength $cdc_missing] > 0} {
    append summary "CDC constraint objects NOT FOUND: $cdc_missing\n\n"
}
append summary [write_reports post_synth $rpt] "\n"
write_checkpoint -force [file join $chk post_synth.dcp]

# ---- implementation ----------------------------------------------------------------
if {$flow eq "impl"} {
    opt_design
    place_design
    phys_opt_design
    route_design
    append summary [write_reports post_route $rpt] "\n"
    write_checkpoint -force [file join $chk post_route.dcp]
}

append summary [format "elapsed %d s\n" [expr {[clock seconds] - $t_start}]]
set fh [open [file join $rpt summary.txt] w]
puts $fh $summary
close $fh

puts "\n=================================================================="
puts $summary
puts "reports: $rpt"
puts "=================================================================="
