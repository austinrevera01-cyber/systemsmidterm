%% Midterm Project Solution - Part I (Sections a through j)
% This script rebuilds the Part I analysis for the Systems midterm project
% using the motor parameters gathered by our group in the file
% "motorstepwithgearbox 1.m".  The focus here is exclusively on
% requirements (a) through (j):
%   a) Derive the transfer function that retains armature inductance.
%   b) Derive the transfer function that neglects inductance and report
%      the effective inertia (J_eff) and damping (b_eff).
%   c) Compile a comparison table between the model parameters and the
%      published datasheet values.
%   d) Quantify the discrepancy between the model and datasheet values in
%      SI units to support the discussion requested in (c).
%   e) Report the eigenvalues and associated time constants for both
%      transfer functions.
%   f) Simulate and compare the 12 V step responses for the models with
%      and without inductance on a single plot.
%   g) Compare the simulated time constant and steady-state speed against
%      the manufacturer specification.
%   h) Discuss when it is reasonable to neglect armature inductance.
%   i) Re-derive the equations of motion when a gearbox with ratio N is
%      attached to the motor and compute the resulting transfer function.
%   j) Provide a framework to compare the gearbox-inclusive model against
%      externally supplied benchmark data (e.g., from run_Indy_car.p or lab
%      measurements).
%
% The calculations rely on Control System Toolbox functionality.

%% Clear workspace graphics and console (safe to rerun sections individually)
close all; clc;

%% Motor parameters from "motorstepwithgearbox 1.m"
params.R      = 0.611;          % Armature resistance [Ohms]
params.L      = 0.000119;       % Armature inductance [H]
params.Kb     = 0.025879;       % Back EMF constant [V/(rad/s)]
params.Ki     = 0.0259;         % Torque constant [N*m/A]
params.Jm     = 3.35e-6;        % Rotor inertia [kg*m^2]
params.bm     = 4.63e-6;        % Rotor viscous friction [N*m*s/rad]
params.Tstall = 1.02;           % Stall torque [N*m]
params.w_nl   = 922.581;        % No-load speed [rad/s]
params.tau_e  = 0.00305;        % Electrical time constant [s]
params.Vs     = 12;             % Nominal comparison voltage [V]

gear.N        = 299/14;         % Gear ratio (motor:output)
gear.Jload    = 8.0e-7;         % Load inertia at gearbox output [kg*m^2]
gear.encoder_counts = 2000;     % Counts per motor revolution (500 CPT, quadrature)

results_part1 = struct();

%% Part I.a - Transfer function including inductance
num_with_L = params.Ki;
den_with_L = [params.Jm*params.L, ...
              params.bm*params.L + params.Jm*params.R, ...
              params.bm*params.R + params.Ki*params.Kb];
G_omega_with_L = tf(num_with_L, den_with_L, 'Variable', 's');

disp('Part I.a: Transfer function (omega/V) including inductance');
display(G_omega_with_L);

%% Part I.b - Transfer function neglecting inductance
num_no_L = params.Ki;
den_no_L = [params.Jm*params.R, params.bm*params.R + params.Ki*params.Kb];
G_omega_no_L = tf(num_no_L, den_no_L, 'Variable', 's');

J_eff = params.Jm;
b_eff = params.bm + (params.Ki*params.Kb)/params.R;

fprintf('Part I.b: J_eff = %.4e kg*m^2, b_eff = %.4e N*m*s/rad\n', J_eff, b_eff);
disp('Part I.b: Transfer function (omega/V) without inductance');
display(G_omega_no_L);

%% Part I.c - Datasheet lookup values and SI conversions
% Datasheet information for the 24 V Maxon RE-30 (part 268214)
datasheet_table = table( ...
    [0.0987; 0.0987; 0.86; 1.02; 9200; 3.5; 0.00305], ...
    'VariableNames', {'MaxonUnits'}, ...
    'RowNames', {'Kb (V/krpm)','Ki (mN*m/A)','R (Ohm)','T_stall (mN*m)', ...
                 'omega_nl (rpm)','Jm (g*cm^2)','tau_e (s)'});

% Convert datasheet values to SI for comparison
Kb_SI   = datasheet_table{'Kb (V/krpm)','MaxonUnits'} / (60/1000); % V/(rad/s)
Ki_SI   = datasheet_table{'Ki (mN*m/A)','MaxonUnits'} / 1000;      % N*m/A
R_SI    = datasheet_table{'R (Ohm)','MaxonUnits'};                 % Ohms
Tstall_SI = datasheet_table{'T_stall (mN*m)','MaxonUnits'} / 1000; % N*m
omega_nl_SI = datasheet_table{'omega_nl (rpm)','MaxonUnits'} * 2*pi/60; % rad/s
Jm_SI   = datasheet_table{'Jm (g*cm^2)','MaxonUnits'} * 1e-7;      % kg*m^2
tau_e_SI = datasheet_table{'tau_e (s)','MaxonUnits'};              % seconds

datasheet_SI = table([Kb_SI; Ki_SI; R_SI; Tstall_SI; omega_nl_SI; Jm_SI; tau_e_SI], ...
    'VariableNames', {'Value_SI'}, ...
    'RowNames', {'Kb','Ki','R','T_stall','omega_nl','Jm','tau_e'});

model_vs_datasheet = table( ...
    [params.Kb; params.Ki; params.R; params.Tstall; params.w_nl; params.Jm; params.tau_e], ...
    datasheet_SI.Value_SI, ...
    'VariableNames', {'Model_Value','Datasheet_Value'}, ...
    'RowNames', datasheet_SI.Properties.RowNames);

disp('Part I.c: Model parameters vs datasheet values (SI units)');
disp(model_vs_datasheet);

%% Part I.d - Percent discrepancy between model and datasheet values
percent_error = 100 * (model_vs_datasheet.Model_Value - model_vs_datasheet.Datasheet_Value) ...
    ./ model_vs_datasheet.Datasheet_Value;

comparison_summary = table(model_vs_datasheet.Model_Value, ...
    model_vs_datasheet.Datasheet_Value, percent_error, ...
    'VariableNames', {'Model_Value','Datasheet_Value','Percent_Difference'}, ...
    'RowNames', model_vs_datasheet.Properties.RowNames);

disp('Part I.d: Percent difference between model and datasheet values');
disp(comparison_summary);

%% Part I.e - Eigenvalues and time constants
eig_with_L = pole(G_omega_with_L);
eig_no_L   = pole(G_omega_no_L);

tau_with_L = -1 ./ real(eig_with_L);
tau_no_L   = -1 ./ real(eig_no_L);

fprintf('Part I.e: Eigenvalues with inductance = %s\n', mat2str(eig_with_L,4));
fprintf('          Time constants with inductance = %s s\n', mat2str(tau_with_L,4));
fprintf('          Eigenvalues without inductance = %s\n', mat2str(eig_no_L,4));
fprintf('          Time constants without inductance = %s s\n', mat2str(tau_no_L,4));

%% Part I.f - 12 V step response comparison
time_span = 0:1e-4:0.1; % 0.1 s horizon captures electrical and mechanical dynamics
[y_with_L, t_with_L] = step(params.Vs * G_omega_with_L, time_span);
[y_no_L,   t_no_L]   = step(params.Vs * G_omega_no_L, time_span);

figure('Name','Part I.f Step Response','NumberTitle','off');
plot(t_with_L, y_with_L, 'LineWidth', 1.5); hold on;
plot(t_no_L, y_no_L, '--', 'LineWidth', 1.5);
grid on;
title('Motor Speed Response to 12 V Step');
xlabel('Time [s]');
ylabel('\omega_m [rad/s]');
legend('With inductance','Without inductance','Location','southeast');

stepinfo_with_L = stepinfo(params.Vs * G_omega_with_L);
stepinfo_no_L   = stepinfo(params.Vs * G_omega_no_L);

fprintf('Part I.f: With inductance -> steady-state %.2f rad/s, settling %.4f s\n', ...
    dcgain(params.Vs * G_omega_with_L), stepinfo_with_L.SettlingTime);
fprintf('          Without inductance -> steady-state %.2f rad/s, settling %.4f s\n', ...
    dcgain(params.Vs * G_omega_no_L), stepinfo_no_L.SettlingTime);

%% Part I.g - Agreement with manufacturer specifications
omega_ss_model = dcgain(params.Vs * G_omega_no_L);
omega_ss_datasheet = params.Vs / datasheet_SI{'Kb','Value_SI'};
tau_mech_model = J_eff / b_eff;
% Datasheet viscous coefficient from torque-speed line (T_stall/omega_nl)
b_eff_datasheet = datasheet_SI{'T_stall','Value_SI'} / datasheet_SI{'omega_nl','Value_SI'};
tau_mech_datasheet = datasheet_SI{'Jm','Value_SI'} / b_eff_datasheet;

fprintf('Part I.g: Model steady-state speed = %.2f rad/s, datasheet prediction = %.2f rad/s\n', ...
    omega_ss_model, omega_ss_datasheet);
fprintf('          Model mechanical time constant = %.4f s, datasheet-based estimate = %.4f s\n', ...
    tau_mech_model, tau_mech_datasheet);
fprintf('          Steady-state difference = %.2f%%%%, time constant difference = %.2f%%%%\n', ...
    100*(omega_ss_model - omega_ss_datasheet)/omega_ss_datasheet, ...
    100*(tau_mech_model - tau_mech_datasheet)/tau_mech_datasheet);

results_part1.omega_ss_model = omega_ss_model;
results_part1.omega_ss_datasheet = omega_ss_datasheet;
results_part1.tau_mech_model = tau_mech_model;
results_part1.tau_mech_datasheet = tau_mech_datasheet;
results_part1.omega_ss_percent_diff = 100*(omega_ss_model - omega_ss_datasheet)/omega_ss_datasheet;
results_part1.tau_mech_percent_diff = 100*(tau_mech_model - tau_mech_datasheet)/tau_mech_datasheet;

%% Part I.h - Reasonableness of neglecting inductance
tau_electrical = params.L / params.R;
ratio_tau = tau_electrical / tau_mech_model;

fprintf('Part I.h: Electrical time constant tau_e = %.4e s, mechanical tau_m = %.4e s (ratio = %.3f)\n', ...
    tau_electrical, tau_mech_model, ratio_tau);
if ratio_tau < 0.1
    fprintf('          Inductance can be neglected for slower mechanical studies; faster inputs require the inductive model.\n');
else
    fprintf('          Inductance materially influences the response and should be retained.\n');
end

results_part1.tau_electrical = tau_electrical;
results_part1.tau_ratio = ratio_tau;

%% Part I.i - Motor with gearbox equations of motion
J_total = params.Jm + gear.Jload * gear.N^2;
den_with_L_gear = [J_total * params.L, ...
                   params.bm * params.L + J_total * params.R, ...
                   params.bm * params.R + params.Ki * params.Kb];
G_omega_with_L_gear = tf(params.Ki, den_with_L_gear, 'Variable', 's');
G_omega_out_with_L = (1/gear.N) * G_omega_with_L_gear;

fprintf('Part I.i: Effective inertia with gearbox = %.4e kg*m^2\n', J_total);
disp('          Transfer function V -> omega_motor with gearbox (includes inductance):');
display(G_omega_with_L_gear);
disp('          Transfer function V -> omega_output with gearbox (includes inductance):');
display(G_omega_out_with_L);

time_span_gear = 0:1e-3:2; % Gearbox dynamics are slower; extend the horizon
[omega_out_step, t_gear] = step(params.Vs * G_omega_out_with_L, time_span_gear);

figure('Name','Part I.i Gearbox Motor Model','NumberTitle','off');
plot(t_gear, omega_out_step, 'LineWidth', 1.5);
grid on;
title('Motor Model with Gearbox (12 V Step)');
xlabel('Time [s]');
ylabel('\omega_{out} [rad/s]');

results_part1.gear_step_time = t_gear(:);
results_part1.gear_step_response = omega_out_step(:);

results_part1.J_total             = J_total;
results_part1.G_omega_with_L_gear = G_omega_with_L_gear;
results_part1.G_omega_out_with_L  = G_omega_out_with_L;
results_part1.gear_ratio          = gear.N;
results_part1.gear_load_inertia   = gear.Jload;
results_part1.gear_encoder_counts = gear.encoder_counts;

%% Part I.j - Framework for external gearbox benchmark comparison
results_part1.gearbox_comparison = table([], [], [], ...
    'VariableNames', {'Time_s','Omega_model_rad_s','Omega_reference_rad_s'});
results_part1.gearbox_error = '';

% The analytical gearbox response to a constant-voltage command is readily
% available via G_omega_out_with_L.  If a reference dataset is available (for
% example, lab measurements or the proprietary run_Indy_car.p output), supply
% it in a struct named `gearbox_reference` with fields:
%   gearbox_reference.Time_s             -> column vector of time stamps [s]
%   gearbox_reference.VoltageCommand_V   -> column vector of voltage inputs [V]
%   gearbox_reference.Omega_rad_s        -> column vector of output speeds [rad/s]
% The code below will align the sample times, simulate the analytical model,
% and assemble a comparison table and plot.  If the struct is absent, the
% section simply reports the instructions above.

if exist('gearbox_reference','var')
    try
        clear run_Indy_car; %#ok<CLRUN>

        % Match the data-collection approach used in run_Indy_car_12V_demo.m
        V_step  = params.Vs;
        Ts      = 1e-3;        % Sample time enforced by the p-code interface
        T_final = 0.5;         % Capture both transient and steady-state regions
        CPR     = gear.encoder_counts; % Quadrature counts per revolution
        MAXCNT  = 4096;        % Encoder rollover value from the demo script

        steps    = round(T_final / Ts);
        t_compare = (0:steps-1).' * Ts;
        u_compare = V_step * ones(size(t_compare));

        % Initialize the p-code persistent state as in the demo
        Vel       = 0;
        X0_values = [0 0 0 0 0];
        WP_FILE   = 0;
        [~, ~, ~] = run_Indy_car(0, Vel, X0_values, WP_FILE);

        % Collect encoder counts with rollover protection
        acc_counts   = 0;
        last_raw     = NaN;
        theta_counts = zeros(steps, 1);
        for k = 1:steps
            [~, ~, counts_k] = run_Indy_car(V_step);
            raw = double(counts_k);
            if isnan(last_raw)
                acc_counts = raw;
            else
                d = raw - last_raw;
                if d >  MAXCNT / 2, d = d - MAXCNT; end
                if d < -MAXCNT / 2, d = d + MAXCNT; end
                acc_counts = acc_counts + d;
            end
            last_raw = raw;
            theta_counts(k) = acc_counts;
        end

        % Convert counts to motor angle and unwrap
        theta_motor = unwrap(theta_counts * (2*pi / CPR));
        theta_output = theta_motor / gear.N;

        % Central-difference differentiation for speed estimate
        omega_output_pcode = zeros(steps, 1);
        if steps >= 3
            omega_output_pcode(2:end-1) = (theta_output(3:end) - theta_output(1:end-2)) / (2 * Ts);
            omega_output_pcode(1)       = (theta_output(2) - theta_output(1)) / Ts;
            omega_output_pcode(end)     = (theta_output(end) - theta_output(end-1)) / Ts;
        end

        % Analytical gearbox model simulated on the same time grid
        omega_model = lsim(G_omega_out_with_L, u_compare, t_compare);
        omega_model = omega_model(:);

        % Steady-state and time-constant verification
        tail_count = min(steps, max(5, round(0.1 * steps)));
        omega_model_ss = mean(omega_model(end-tail_count+1:end));
        omega_pcode_ss = mean(omega_output_pcode(end-tail_count+1:end));
        tau_target_model = omega_model_ss * (1 - exp(-1));
        tau_target_pcode = omega_pcode_ss * (1 - exp(-1));
        idx_tau_model = find(omega_model >= tau_target_model, 1, 'first');
        idx_tau_pcode = find(omega_output_pcode >= tau_target_pcode, 1, 'first');
        tau_model = NaN;
        tau_pcode = NaN;
        if ~isempty(idx_tau_model)
            tau_model = t_compare(idx_tau_model);
        end
        if ~isempty(idx_tau_pcode)
            tau_pcode = t_compare(idx_tau_pcode);
        end
        tau_mech_analytic = J_total / (params.bm + (params.Ki * params.Kb) / params.R);
        rmse_compare = sqrt(mean((omega_model - omega_output_pcode).^2));
        tau_percent_diff = NaN;
        if ~isnan(tau_model) && ~isnan(tau_pcode) && tau_pcode ~= 0
            tau_percent_diff = 100 * (tau_model - tau_pcode) / tau_pcode;
        end

        fprintf(['Part I.j: Gearbox steady-state speed (model vs p-code) = %.3f vs %.3f rad/s, ', ...
                 'RMSE = %.3f rad/s\n'], omega_model_ss, omega_pcode_ss, rmse_compare);
        fprintf(['          Time constant from data (model vs p-code) = %.4f vs %.4f s (%.2f%%%% difference); ', ...
                 'analytic mechanical tau = %.4f s\n'], tau_model, tau_pcode, tau_percent_diff, tau_mech_analytic);

        % Visualization
        figure('Name','Part I.j Gearbox Comparison','NumberTitle','off');
        plot(t_compare, omega_model, 'LineWidth', 1.5); hold on;
        plot(t_compare, omega_output_pcode, '--', 'LineWidth', 1.5);
        grid on;
        title('Steering Output Speed Benchmark');
        xlabel('Time [s]');
        ylabel('\omega_{tire} [rad/s]');
        legend('Analytical gearbox model','Reference data','Location','southeast');

        % Store comparison results
        results_part1.gearbox_comparison = table(t_compare, omega_model, omega_output_pcode, ...
            'VariableNames', {'Time_s','Omega_model_rad_s','Omega_pcode_rad_s'});
        results_part1.gearbox_motor_comparison = table(t_compare, omega_model_motor, omega_motor_pcode, ...
            'VariableNames', {'Time_s','Omega_motor_model_rad_s','Omega_motor_pcode_rad_s'});
        results_part1.gearbox_error = '';
        results_part1.gearbox_tau_model = tau_model;
        results_part1.gearbox_tau_pcode = tau_pcode;
        results_part1.gearbox_tau_analytic = tau_mech_analytic;
        results_part1.gearbox_tau_percent_diff = tau_percent_diff;
        results_part1.gearbox_rmse = rmse_compare;
        results_part1.gearbox_ss_model = omega_model_ss;
        results_part1.gearbox_ss_pcode = omega_pcode_ss;

        % Close any file handles opened by run_Indy_car
        fclose('all');
    catch err
        warning('Part I.j comparison skipped due to error: %s', err.message);
        results_part1.gearbox_error = err.message;
    end
else
    disp('Part I.j: Provide a struct named gearbox_reference with fields Time_s,');
    disp('           VoltageCommand_V, and Omega_rad_s to generate the comparison plot.');
end

%% Export summary for quick access (optional convenience for MATLAB users)
results_part1.G_omega_with_L = G_omega_with_L;
results_part1.G_omega_no_L   = G_omega_no_L;
results_part1.comparison     = comparison_summary;
results_part1.stepinfo_with_L = stepinfo_with_L;
results_part1.stepinfo_no_L   = stepinfo_no_L;
assignin('base','midterm_part1_results',results_part1);

disp('Part I (a-j) analysis complete. Results available in midterm_part1_results.');
return;
