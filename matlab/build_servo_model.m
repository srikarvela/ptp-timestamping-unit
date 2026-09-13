function mdl = build_servo_model(P)
% BUILD_SERVO_MODEL  Programmatically create ptp_servo.slx.
%
%   drift_ppb ──(+)──▶ [1/s]  offset_ns ──(+)──▶ meas_ns ──┬─▶ Kp ──(+)──▶ corr_ppb
%               (-)      FE DTI            (+)              └─▶ Ki ▶ [1/s] ┘
%                ▲                          pdv_ns (From Workspace)        │
%                └─────────────────────────────────────────────────────────┘
%
% Units: offsets in ns, rates in ppb, so 1 ppb over 1 s = 1 ns and the
% plant integrator needs no scaling.  Discrete, sample time P.Ts.
%
% Both integrators are Forward-Euler Discrete-Time Integrators, i.e.
%   y(k+1) = y(k) + Ts*u(k)
% which is exactly the update the bit-exact vector generator uses, so the
% two trajectories can be compared to sub-ns precision.

    mdl = 'ptp_servo';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
    new_system(mdl);

    add_block('simulink/Sources/Step', [mdl '/drift_ppb'], ...
        'Time', 'P.drift_step_k*P.Ts', 'Before', 'P.drift_ppb', ...
        'After', 'P.drift_ppb + P.drift_step_ppb', 'SampleTime', 'P.Ts', ...
        'Position', [40 80 70 110]);
    add_block('simulink/Sources/From Workspace', [mdl '/pdv_ns'], ...
        'VariableName', 'pdv_ts', 'SampleTime', 'P.Ts', 'Interpolate', 'off', ...
        'OutputAfterFinalValue', 'Holding final value', 'Position', [300 160 360 190]);

    add_block('simulink/Math Operations/Sum', [mdl '/ferr'], 'Inputs', '+-', ...
        'Position', [130 85 150 105]);
    add_block('simulink/Discrete/Discrete-Time Integrator', [mdl '/slave_clock'], ...
        'IntegratorMethod', 'Integration: Forward Euler', 'gainval', '1', ...
        'InitialCondition', 'P.init_offset_sim_ns', 'SampleTime', 'P.Ts', ...
        'Position', [200 80 240 110]);
    add_block('simulink/Math Operations/Sum', [mdl '/meas'], 'Inputs', '++', ...
        'Position', [400 85 420 105]);

    add_block('simulink/Math Operations/Gain', [mdl '/Kp'], 'Gain', 'P.Kp', ...
        'Position', [480 60 510 90]);
    add_block('simulink/Math Operations/Gain', [mdl '/Ki'], 'Gain', 'P.Ki', ...
        'Position', [480 120 510 150]);
    add_block('simulink/Discrete/Discrete-Time Integrator', [mdl '/integ'], ...
        'IntegratorMethod', 'Integration: Forward Euler', 'gainval', '1', ...
        'InitialCondition', '0', 'SampleTime', 'P.Ts', 'Position', [550 120 590 150]);
    add_block('simulink/Math Operations/Sum', [mdl '/corr'], 'Inputs', '++', ...
        'Position', [640 85 660 105]);

    add_block('simulink/Sinks/To Workspace', [mdl '/offset_out'], 'VariableName', 'offset_ns', ...
        'SaveFormat', 'Array', 'SampleTime', 'P.Ts', 'Position', [300 20 360 50]);
    add_block('simulink/Sinks/To Workspace', [mdl '/meas_out'], 'VariableName', 'meas_ns', ...
        'SaveFormat', 'Array', 'SampleTime', 'P.Ts', 'Position', [480 20 540 50]);
    add_block('simulink/Sinks/To Workspace', [mdl '/corr_out'], 'VariableName', 'corr_ppb', ...
        'SaveFormat', 'Array', 'SampleTime', 'P.Ts', 'Position', [720 60 780 90]);

    add_line(mdl, 'drift_ppb/1',   'ferr/1');
    add_line(mdl, 'ferr/1',        'slave_clock/1');
    add_line(mdl, 'slave_clock/1', 'meas/1');
    add_line(mdl, 'slave_clock/1', 'offset_out/1');
    add_line(mdl, 'pdv_ns/1',      'meas/2');
    add_line(mdl, 'meas/1',        'Kp/1');
    add_line(mdl, 'meas/1',        'Ki/1');
    add_line(mdl, 'meas/1',        'meas_out/1');
    add_line(mdl, 'Ki/1',          'integ/1');
    add_line(mdl, 'Kp/1',          'corr/1');
    add_line(mdl, 'integ/1',       'corr/2');
    add_line(mdl, 'corr/1',        'ferr/2');
    add_line(mdl, 'corr/1',        'corr_out/1');

    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
        'FixedStep', 'P.Ts', 'StopTime', 'P.T_stop', 'SaveTime', 'on', 'TimeSaveName', 'tout');
    save_system(mdl, [pwd filesep mdl '.slx']);
end
