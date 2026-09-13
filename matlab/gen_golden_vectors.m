function R = gen_golden_vectors(P, pdv_ns)
% GEN_GOLDEN_VECTORS  Run the PI servo against the bit-exact clock model and
% write the vectors the RTL testbench replays.
%
% Per step k (k = 0..K-1):
%   offset_k = slave_time - master_time         (real ns, incl. fraction)
%   if |offset_k| > thresh : phase step  adj = -round(offset_k), integrator reset
%   meas_k   = offset_k + pdv_k
%   integ_k  = integ_{k-1} + Ki*Ts*meas_{k-1}   (Forward Euler, as in Simulink)
%   corr_k   = Kp*meas_k + integ_k              [ppb]
%   F_k      = round(-corr_k * 1e-9 * 6.4 ns * 2^32)   -> FREQ_ADJ register
%   clock advances N cycles with F_k (and the one-shot adj)  -> expected state
%
% The oscillator drift lives on the MASTER side of the model: the RTL clock
% ticks at exactly 6.4 ns in simulation, so "slave runs fast by d" (offset
% = slave - master grows by d*Ts per step, as in the Simulink plant) is the
% same as "master advances only Ts*(1-d) per step".
%
% Vector file: one line per step, 9 hex tokens
%   F(32) ADJ_VALID ADJ_SEC(32) ADJ_NS(32) N(32) EXP_SEC(48) EXP_NS(32) EXP_FRAC(32) MASTER_END_NS(64)

    K = P.K;  N = P.N;  Ts_ns = P.Ts * 1e9;
    st = ptp_clock_init(P.slave_sec0, P.slave_ns0, P.nom_incr);
    master_ns = P.slave_sec0 * 1e9 + P.slave_ns0 - P.init_offset_ns;
    integ = 0;  meas_prev = 0;

    R.k = (0:K-1)';  R.t = R.k * P.Ts;
    R.offset_ns = zeros(K,1);  R.meas_ns = zeros(K,1);  R.corr_ppb = zeros(K,1);
    R.F = zeros(K,1);  R.adj_valid = zeros(K,1);  R.adj_total_ns = zeros(K,1);
    R.exp_sec = zeros(K,1); R.exp_ns = zeros(K,1); R.exp_frac = zeros(K,1); R.master_end = zeros(K,1);

    fid = fopen(fullfile('vectors', 'servo_vectors.txt'), 'w');
    fprintf(fid, '%08X\n', K);
    for i = 1:K
        k = i - 1;
        drift = P.drift_ppb + (k >= P.drift_step_k) * P.drift_step_ppb;

        slave_ns = st.sec * 1e9 + st.ns + double(st.frac) / 2^32;
        offset   = slave_ns - master_ns;

        adj_valid = 0; adj_sec = 0; adj_ns = 0;
        if abs(offset) > P.step_thresh_ns
            adj_total = -round(offset);
            adj_sec   = fix(adj_total / 1e9);
            adj_ns    = adj_total - adj_sec * 1e9;
            adj_valid = 1;
            integ = 0; meas_prev = 0;
            offset = offset + adj_total;            % sub-ns residual after the step
            R.adj_total_ns(i) = adj_total;
        end

        meas  = offset + pdv_ns(i);
        integ = integ + P.Ki * P.Ts * meas_prev;
        corr  = P.Kp * meas + integ;
        meas_prev = meas;

        F = round(-corr * 1e-9 * P.period_ns * 2^P.frac_bits);
        F = max(min(F, 2^31 - 1), -2^31);

        st = ptp_clock_step(st, F, adj_valid, adj_sec, adj_ns, N, P.nom_incr);
        master_ns = master_ns + Ts_ns * (1 - drift * 1e-9);

        R.offset_ns(i) = offset;  R.meas_ns(i) = meas;  R.corr_ppb(i) = corr;
        R.F(i) = F;  R.adj_valid(i) = adj_valid;
        R.exp_sec(i) = st.sec;  R.exp_ns(i) = st.ns;  R.exp_frac(i) = double(st.frac);
        R.master_end(i) = round(master_ns);

        fprintf(fid, '%08X %X %08X %08X %08X %012X %08X %08X %016X\n', ...
            u32(F), adj_valid, u32(adj_sec), u32(adj_ns), N, ...
            st.sec, st.ns, double(st.frac), uint64(round(master_ns)));
    end
    fclose(fid);

    % final residual offset (after the last step) for reporting
    slave_ns = st.sec * 1e9 + st.ns + double(st.frac) / 2^32;
    R.final_offset_ns = slave_ns - master_ns;
end

function v = u32(x)
    v = mod(x, 2^32);
end
