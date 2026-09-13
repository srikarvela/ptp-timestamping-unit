function st = ptp_clock_init(sec, ns, nom_incr)
% PTP_CLOCK_INIT  Bit-exact model of ptp_clock_core state after a SET load.
%   sec, ns   : loaded time (frac cleared, as in the RTL)
%   nom_incr  : uint64 Q8.32 nominal increment
%
% Mirrors the RTL's one-cycle-registered increment: after reset + SET with
% freq_adj = 0, incr_eff == nom_incr.
    st.sec       = double(sec);
    st.ns        = double(ns);
    st.frac      = uint64(0);
    st.incr_prev = uint64(nom_incr);      % incr_eff currently registered in the RTL
end
