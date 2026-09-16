# =============================================================================
# ptp_tsu_clocks.xdc -- clocks for out-of-context synthesis of ptp_tsu_top
#
# Read before synth_design.  The two clocks are unrelated; every path between
# them goes through a handshake synchroniser and is constrained explicitly in
# ptp_cdc.xdc (sourced after synthesis).  We deliberately do NOT use
# set_clock_groups -asynchronous: it would override the set_max_delay /
# set_bus_skew bounds the handshake relies on.
#
# Only register-to-register paths are timed.  In an OOC run the ports have no
# input/output delays, so Fmax below is the internal reg-to-reg figure.
# =============================================================================

create_clock -name net_clk -period 6.400  [get_ports net_clk]
create_clock -name aclk    -period 10.000 [get_ports aclk]
