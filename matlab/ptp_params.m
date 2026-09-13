function P = ptp_params()
% PTP_PARAMS  Shared parameters for the servo model, the golden-vector
% generator and the analysis script.
%
% The loop runs at a 1 ms sync interval (scaled down from the 125 ms .. 1 s
% intervals real PTP uses) so that the RTL replay of the vectors fits in a
% few tens of seconds of Icarus time: 300 steps x 156 250 clock cycles.
% Gains are chosen for the same closed-loop shape a real servo would have,
% just at a higher natural frequency; the loop is scale-invariant in
% omega_n * Ts.

    % ---- hardware --------------------------------------------------------
    P.f_clk       = 156.25e6;             % network clock
    P.period_ns   = 6.4;                  % nominal increment (Q8.32 in RTL)
    P.frac_bits   = 32;
    P.nom_incr    = uint64(hex2dec('666666666'));   % floor(6.4 * 2^32)

    % ---- servo timing ------------------------------------------------------
    P.Ts          = 1e-3;                 % sync interval [s]
    P.N           = round(P.Ts * P.f_clk);% clock cycles per step (156 250)
    P.K           = 300;                  % number of servo steps (0.3 s)
    P.T_stop      = (P.K - 1) * P.Ts;

    % ---- PI controller  corr_ppb = Kp*offset_ns + Ki*Ts*sum(offset_ns) ----
    % plant: offset' = drift - corr  ->  closed loop s^2 + Kp s + Ki
    %   omega_n = sqrt(Ki) = 2*pi*10 rad/s,  zeta = Kp / (2 sqrt(Ki)) = 0.70
    P.Kp          = 88;                   % [1/s]
    P.Ki          = 3948;                 % [1/s^2]
    P.step_thresh_ns = 1e6;               % |offset| above this -> phase step, not slew

    % ---- disturbance / scenario --------------------------------------------
    P.drift_ppb      = 37500;             % slave oscillator error (+37.5 ppm, runs fast)
    P.drift_step_k   = 150;               % at step 150 the oscillator drifts further
    P.drift_step_ppb = 12000;             % (+12 ppm, e.g. a temperature transient)
    P.pdv_sigma_ns   = 50;                % packet delay variation, Gaussian, 1-sigma
    P.init_offset_ns = 2.5e6;             % slave starts 2.5 ms ahead of the master
    P.seed           = 1588;

    % ---- initial slave time loaded into the RTL --------------------------
    P.slave_sec0  = 1000;
    P.slave_ns0   = 0;
end
