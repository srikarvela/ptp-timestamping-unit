# =============================================================================
# ptp_cdc.xdc -- clock-domain-crossing constraints for ptp_tsu_top
#
# Sourced by ooc_synth.tcl after synth_design, on the synthesised netlist, so
# the register names below are post-synthesis names (<signal>_reg[<bit>]).
# ooc_synth.tcl checks that every object matches before sourcing this file.
#
# Two crossings, same pattern (see rtl/ptp_time_snapshot.sv header):
#   - single-bit toggles cross through ASYNC_REG 2-flop synchronisers
#       -> false path into the first synchroniser stage
#   - multi-bit payloads are frozen while their toggle is in flight and are
#     sampled only after the synchronised toggle arrives (multi-cycle path)
#       -> set_max_delay -datapath_only + set_bus_skew of one net_clk period,
#          the tightest bound the handshake needs (the toggle takes at least
#          two capture-clock edges to arrive)
# =============================================================================

set CDC_BOUND 6.400

# ---- u_snap : ptp_time_snapshot (net_clk -> aclk time read) ------------------
set_false_path -from [get_cells u_snap/req_tgl_reg] -to [get_cells {u_snap/req_sync_reg[0]}]
set_false_path -from [get_cells u_snap/ack_tgl_reg] -to [get_cells {u_snap/ack_sync_reg[0]}]

set_max_delay -datapath_only \
    -from [get_cells {u_snap/snap_reg[*]}] \
    -to   [get_cells {u_snap/dst_sec_reg[*] u_snap/dst_ns_reg[*]}] $CDC_BOUND
set_bus_skew \
    -from [get_cells {u_snap/snap_reg[*]}] \
    -to   [get_cells {u_snap/dst_sec_reg[*] u_snap/dst_ns_reg[*]}] $CDC_BOUND

# ---- u_bridge : cdc_bus_bridge (aclk <-> net_clk register access) ------------
set_false_path -from [get_cells u_bridge/req_tgl_reg] -to [get_cells {u_bridge/req_sync_reg[0]}]
set_false_path -from [get_cells u_bridge/ack_tgl_reg] -to [get_cells {u_bridge/ack_sync_reg[0]}]

# request payload: aclk -> net_clk
set_max_delay -datapath_only \
    -from [get_cells {u_bridge/addr_hold_reg[*] u_bridge/wdata_hold_reg[*] u_bridge/we_hold_reg}] \
    -to   [get_cells {u_bridge/s_addr_reg[*] u_bridge/s_wdata_reg[*] u_bridge/s_we_reg}] $CDC_BOUND
set_bus_skew \
    -from [get_cells {u_bridge/addr_hold_reg[*] u_bridge/wdata_hold_reg[*] u_bridge/we_hold_reg}] \
    -to   [get_cells {u_bridge/s_addr_reg[*] u_bridge/s_wdata_reg[*] u_bridge/s_we_reg}] $CDC_BOUND

# response payload: net_clk -> aclk
set_max_delay -datapath_only \
    -from [get_cells {u_bridge/rdata_hold_reg[*]}] \
    -to   [get_cells {u_bridge/m_rdata_reg[*]}] $CDC_BOUND
set_bus_skew \
    -from [get_cells {u_bridge/rdata_hold_reg[*]}] \
    -to   [get_cells {u_bridge/m_rdata_reg[*]}] $CDC_BOUND
