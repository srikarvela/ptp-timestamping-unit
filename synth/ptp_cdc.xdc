# =============================================================================
# ptp_cdc.xdc -- clock-domain-crossing constraints for ptp_time_snapshot
#
# Scope these to the instance with:  read_xdc -ref ptp_time_snapshot ptp_cdc.xdc
# (or  set_property SCOPED_TO_REF ptp_time_snapshot [get_files ptp_cdc.xdc]).
#
# The two toggle flags cross through ASYNC_REG synchronisers; the 80-bit
# snapshot bus crosses as a multi-cycle path qualified by the synchronised
# ack.  We do NOT false-path the bus outright: a datapath-only max delay of
# one source period keeps the bus skew smaller than the >= 2 dst cycles the
# ack takes, which is the assumption the handshake relies on.
# =============================================================================

# --- toggle flags: single-bit, synchroniser handles them ----------------------
set_false_path -from [get_cells req_tgl_reg]  -to [get_cells {req_sync_reg[0]}]
set_false_path -from [get_cells ack_tgl_reg]  -to [get_cells {ack_sync_reg[0]}]

# --- 80-bit snapshot bus: src-domain 'snap' -> dst-domain copy ---------------
# Bound the datapath to one source clock period and the bus skew to the same.
set_max_delay -datapath_only \
    -from [get_cells {snap_reg[*]}] \
    -to   [get_cells {dst_sec_reg[*] dst_ns_reg[*]}] \
    [expr {1.0 * [get_property PERIOD [get_clocks -of_objects [get_pins snap_reg[0]/C]]]}]
set_bus_skew \
    -from [get_cells {snap_reg[*]}] \
    -to   [get_cells {dst_sec_reg[*] dst_ns_reg[*]}] \
    [expr {1.0 * [get_property PERIOD [get_clocks -of_objects [get_pins snap_reg[0]/C]]]}]
