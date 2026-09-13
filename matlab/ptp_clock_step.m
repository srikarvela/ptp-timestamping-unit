function st = ptp_clock_step(st, F, adj_valid, adj_sec, adj_ns, N, nom_incr)
% PTP_CLOCK_STEP  Advance the bit-exact clock model by N clock cycles.
%
% Cycle-exact semantics of the RTL testbench protocol (tb_servo_golden):
%   - at the step boundary freq_adj = F is driven and, if adj_valid, the
%     offset (adj_sec, adj_ns) is presented for exactly one cycle;
%   - the RTL registers nom_incr + freq_adj one cycle later, so the FIRST
%     of the N cycles still uses the previous increment and the remaining
%     N-1 use the new one;
%   - the offset lands in that first cycle.
%
% Because the RTL's split {sec, ns, frac} arithmetic is exactly equal to a
% flat fixed-point accumulator (proven by tb_ptp_clock_core), N cycles can
% be fast-forwarded in closed form with integer math and stay bit-exact.

    two32 = uint64(4294967296);
    mask  = uint64(4294967295);

    % incr_eff for this step (nom_incr + F, F signed, result always > 0)
    if F >= 0
        incr_new = nom_incr + uint64(F);
    else
        incr_new = nom_incr - uint64(-F);
    end

    % total fractional-ns advance over N cycles
    adv = st.incr_prev + uint64(N - 1) * incr_new;    % < 2^58, no overflow
    tot = st.frac + adv;
    ns_adv  = idivide(tot, two32, 'floor');
    st.frac = bitand(tot, mask);

    st.ns  = st.ns + double(ns_adv);
    if adj_valid
        st.ns  = st.ns  + double(adj_ns);
        st.sec = st.sec + double(adj_sec);
    end
    % normalise ns into [0, 1e9)
    while st.ns >= 1e9, st.ns = st.ns - 1e9; st.sec = st.sec + 1; end
    while st.ns <  0,   st.ns = st.ns + 1e9; st.sec = st.sec - 1; end

    st.incr_prev = incr_new;
end
