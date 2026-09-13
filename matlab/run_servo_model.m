% RUN_SERVO_MODEL  Step 5 driver:  build the Simulink servo, simulate it,
% generate bit-exact golden vectors from the MATLAB clock model with the
% same controller, cross-check the two trajectories, and plot convergence.
%
%   matlab -batch run_servo_model        (from the matlab/ directory)
%
% Outputs
%   ptp_servo.slx                   the Simulink model (regenerated)
%   vectors/servo_vectors.txt       replayed by tb/tb_servo_golden.sv
%   vectors/servo_trajectory.csv    per-step offsets (script + Simulink), corr, F
%   ../docs/servo_convergence.png   convergence plots

clear; clc;
cd(fileparts(mfilename('fullpath')));
P = ptp_params();

% ---- loop-shape analysis (Control System Toolbox) ------------------------
s  = tf('s');
L  = (P.Kp + P.Ki / s) / s;                 % PI * clock-integrator plant
T  = feedback(L, 1);
[wn, zeta] = damp(T);
Td = c2d(T, P.Ts, 'tustin');
fprintf('PI servo:  Kp = %g 1/s, Ki = %g 1/s^2\n', P.Kp, P.Ki);
fprintf('  closed loop  omega_n = %.1f rad/s (%.1f Hz),  zeta = %.2f\n', wn(1), wn(1)/2/pi, zeta(1));
fprintf('  discrete poles |z| = %s  (Ts = %g s)\n', mat2str(abs(pole(Td))', 4), P.Ts);
S = stepinfo(T);
fprintf('  step response: rise %.1f ms, settle (2%%) %.1f ms, overshoot %.1f %%\n', ...
    S.RiseTime*1e3, S.SettlingTime*1e3, S.Overshoot);

% ---- shared PDV sequence -------------------------------------------------
rng(P.seed);
pdv_ns = P.pdv_sigma_ns * randn(P.K, 1);

% ---- bit-exact vectors ---------------------------------------------------
R = gen_golden_vectors(P, pdv_ns);
fprintf('golden vectors: %d steps x %d cycles = %.1f M clock cycles, %d phase step(s)\n', ...
    P.K, P.N, P.K * P.N / 1e6, sum(R.adj_valid));

% ---- Simulink ------------------------------------------------------------
% The Simulink model has no phase-step logic: it starts where the script
% is right after its step-0 phase step (a sub-ns residual).
P.init_offset_sim_ns = R.offset_ns(1);
pdv_ts = timeseries(pdv_ns, R.t);
assignin('base', 'P', P);
assignin('base', 'pdv_ts', pdv_ts);
mdl = build_servo_model(P);
out = sim(mdl, 'StopTime', num2str(P.T_stop));
sim_offset = out.offset_ns(:);
sim_corr   = out.corr_ppb(:);
close_system(mdl, 0);

% ---- cross-check -----------------------------------------------------------
n = min(numel(sim_offset), P.K);
d_off  = R.offset_ns(1:n) - sim_offset(1:n);
d_corr = R.corr_ppb(1:n)  - sim_corr(1:n);
fprintf('Simulink vs bit-exact model over %d steps:\n', n);
fprintf('  max |offset diff| = %.4f ns,  max |corr diff| = %.4f ppb\n', max(abs(d_off)), max(abs(d_corr)));
fprintf('  (differences come only from FREQ_ADJ quantisation, 0.036 ppb/LSB)\n');

% ---- lock statistics -------------------------------------------------------
lock_thr = 100;                                    % ns
locked = abs(R.offset_ns) < lock_thr;
k_lock = find(~locked(1:P.drift_step_k), 1, 'last') + 1;   % first step of sustained lock
seg = R.offset_ns(k_lock:P.drift_step_k-1);
fprintf('lock: |offset| < %d ns from step %d (t = %.0f ms); residual RMS %.1f ns, peak %.1f ns\n', ...
    lock_thr, k_lock-1, (k_lock-1)*P.Ts*1e3, rms(seg), max(abs(seg)));
seg2 = R.offset_ns(P.drift_step_k:end);
k2 = find(abs(seg2) >= lock_thr, 1, 'last');
if isempty(k2), k2 = 0; end
fprintf('drift step of %+d ppb at step %d: peak excursion %.0f ns, back under %d ns after %d ms\n', ...
    P.drift_step_ppb, P.drift_step_k, max(abs(seg2)), lock_thr, k2);
fprintf('final offset %.3f ns, final FREQ_ADJ %d LSB = %.1f ppb (drift %d ppb)\n', ...
    R.final_offset_ns, R.F(end), -R.F(end) / (P.period_ns * 2^P.frac_bits) * 1e9, P.drift_ppb + P.drift_step_ppb);

% ---- trajectory CSV ----------------------------------------------------------
Tt = table(R.k, R.t, R.offset_ns, [sim_offset(1:n); nan(P.K-n,1)], R.meas_ns, R.corr_ppb, R.F, R.adj_valid, ...
    'VariableNames', {'k','t_s','offset_ns','offset_simulink_ns','meas_ns','corr_ppb','freq_adj','adj_valid'});
writetable(Tt, fullfile('vectors', 'servo_trajectory.csv'));

% ---- plots -------------------------------------------------------------------
f = figure('Visible', 'off', 'Position', [100 100 1000 800], 'Color', 'w');
try, theme(f, 'light'); catch, end                 % R2025a+ dark-theme default
set(f, 'InvertHardcopy', 'off');
t_ms = R.t * 1e3;

subplot(3,1,1);
plot(t_ms, R.offset_ns, 'b-', 'LineWidth', 1.2); hold on;
plot(t_ms(1:n), sim_offset(1:n), 'r--', 'LineWidth', 1.0);
yline(0, 'k:');
xline(P.drift_step_k * P.Ts * 1e3, 'k--', sprintf('+%d ppb drift step', P.drift_step_ppb), 'LabelOrientation', 'horizontal');
ylabel('offset (ns)'); grid on;
legend('bit-exact clock model (golden)', 'Simulink', 'Location', 'northeast');
title(sprintf('PTP PI servo: drift %+d ppb, PDV \\sigma = %d ns, Ts = %g ms, \\omega_n = %.0f rad/s, \\zeta = %.2f', ...
    P.drift_ppb, P.pdv_sigma_ns, P.Ts*1e3, wn(1), zeta(1)));

subplot(3,1,2);
plot(t_ms, R.corr_ppb, 'b-', 'LineWidth', 1.2); hold on;
plot(t_ms(1:n), sim_corr(1:n), 'r--');
yline(P.drift_ppb, 'k:', 'drift'); yline(P.drift_ppb + P.drift_step_ppb, 'k:', 'drift + step');
ylabel('correction (ppb)'); grid on;
legend('golden', 'Simulink', 'Location', 'southeast');

subplot(3,1,3);
plot(t_ms, R.offset_ns, 'b-', 'LineWidth', 1.2); hold on;
plot(t_ms, R.meas_ns, 'Color', [0.6 0.6 0.6]);
ylim([-250 250]); yline(0, 'k:');
xlabel('time (ms)'); ylabel('offset (ns), zoom'); grid on;
legend('true offset', 'measured (offset + PDV)', 'Location', 'northeast');

print(f, fullfile('..', 'docs', 'servo_convergence.png'), '-dpng', '-r110');
close(f);
fprintf('wrote vectors/servo_vectors.txt, vectors/servo_trajectory.csv, docs/servo_convergence.png\n');
