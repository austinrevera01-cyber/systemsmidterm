%% Midterm Project Solution - Part II (Controller Design and Simulation)
% This script addresses the Part II requirements for the Systems midterm
% project.  The goal is to design a controller for the angular position of
% the Maxon RE-30 motor with the GP 32 3:21 gearhead and to evaluate the
% resulting closed-loop performance.
%
%   Part II.a) Design a controller so that the closed-loop system exhibits
%             approximately 5%% overshoot and a 2%% settling time faster than
%             0.025 s.  Report the resulting closed-loop bandwidth.
%   Part II.b) Simulate the controller using the Part I motor model.  Plot
%             the step (constant reference) response as well as the response
%             to a sinusoidal reference.  Compare the simulated performance
%             against the design predictions.  If experimental Bode data are
%             available, overlay the measured frequency response.
%
% The analysis uses Control System Toolbox functionality that is available
% in both MATLAB and Octave (with the control package).

%% Housekeeping
close all; clc;

%% Motor and gearhead parameters (identical to Part I values)
params.R  = 0.611;           % Armature resistance [Ohm]
params.L  = 0.000119;        % Armature inductance [H]
params.Kb = 0.025879;        % Back EMF constant [V/(rad/s)]
params.Ki = 0.0259;          % Torque constant [N*m/A]
params.Jm = 3.35e-6;         % Rotor inertia [kg*m^2]
params.bm = 4.63e-6;         % Rotor viscous friction [N*m*s/rad]

gear.N     = 299/14;         % Gear ratio (motor:output)
gear.Jload = 8.0e-7;         % Load inertia referred to output [kg*m^2]

%% Develop the geared position plant model V(s) -> theta_out(s)
J_total = params.Jm + gear.Jload * gear.N^2;           % Effective inertia

coeff_L2 = J_total * params.L;
coeff_L1 = params.bm * params.L + J_total * params.R;
coeff_L0 = params.bm * params.R + params.Ki * params.Kb;

% Output position transfer function
G_theta = tf(params.Ki, [coeff_L2, coeff_L1, coeff_L0, 0]);

%% Part II.a - PD controller design from desired characteristic polynomial
percent_overshoot = 5;                % Desired percent overshoot [%]
settling_time_2pct = 0.025;           % 2%% settling time requirement [s]

% Convert overshoot specification to damping ratio (second-order assumption)
damping_ratio = sqrt((log(percent_overshoot/100)^2) / ...
    (pi^2 + log(percent_overshoot/100)^2));

% Natural frequency from the 2%% settling time heuristic (4/(zeta*wn))
natural_frequency = 4 / (damping_ratio * settling_time_2pct);

% Enforce a third pole sufficiently fast such that the dominant pair
% achieves the required performance.  Matching the s^2 coefficient yields
% this location exactly for a PD controller structure.
auxiliary_pole = coeff_L1/coeff_L2 - 2*damping_ratio*natural_frequency;

% Solve for the PD gains by matching the closed-loop characteristic
% polynomial (unity feedback).
K_d = (coeff_L2*(natural_frequency^2 + ...
    2*damping_ratio*natural_frequency*auxiliary_pole) - coeff_L0) / params.Ki;
K_p = (coeff_L2 * natural_frequency^2 * auxiliary_pole) / params.Ki;

C_pd = K_d * tf([1 0], 1) + K_p;

% Closed-loop transfer function from reference position to output position
T_cl = feedback(C_pd * G_theta, 1);

% Step response metrics to confirm the design
step_metrics = stepinfo(T_cl);

% Closed-loop bandwidth (-3 dB) using the Control System Toolbox utility
closed_loop_bandwidth = bandwidth(T_cl);

fprintf('Part II.a results\n');
fprintf('  Damping ratio (zeta)        = %.4f\n', damping_ratio);
fprintf('  Natural frequency (wn)      = %.2f rad/s\n', natural_frequency);
fprintf('  Auxiliary pole location     = %.2f rad/s\n', auxiliary_pole);
fprintf('  PD gains: Kp = %.3f, Kd = %.3f\n', K_p, K_d);
fprintf('  Predicted percent overshoot = %.2f%%%%\n', percent_overshoot);
fprintf('  Simulated percent overshoot = %.2f%%%%\n', step_metrics.Overshoot);
fprintf('  Simulated settling time     = %.4f s\n', step_metrics.SettlingTime);
fprintf('  Closed-loop bandwidth       = %.2f rad/s\n', closed_loop_bandwidth);

%% Part II.b - Simulations for constant and sinusoidal references
% Define a modest 0.05 rad (≈2.9 degree) step for position control
t_step = 0:1e-4:0.1;
ref_step = 0.05 * ones(size(t_step));
[theta_step, t_step_out] = lsim(T_cl, ref_step, t_step);

figure('Name','Part II.b - Step Tracking','NumberTitle','off');
plot(t_step_out, ref_step, 'k--', 'LineWidth', 1.2); hold on;
plot(t_step_out, theta_step, 'LineWidth', 1.5);
grid on;
title('Closed-Loop Position Tracking for 0.05 rad Step');
xlabel('Time [s]');
ylabel('\theta_{out} [rad]');
legend('Reference','Output','Location','southeast');

% Sinusoidal reference at 10 Hz (62.83 rad/s)
amp_sine = 0.02;                 % [rad]
omega_sine = 2*pi*10;            % [rad/s]
t_sine = 0:5e-4:0.2;
ref_sine = amp_sine * sin(omega_sine * t_sine);
[theta_sine, t_sine_out] = lsim(T_cl, ref_sine, t_sine);

% Predicted steady-state amplitude from frequency response
[mag_sine, phase_sine] = bode(T_cl, omega_sine);
mag_sine = squeeze(mag_sine);
phase_sine = squeeze(phase_sine);
predicted_amp = mag_sine * amp_sine;

figure('Name','Part II.b - Sinusoidal Tracking','NumberTitle','off');
plot(t_sine_out, ref_sine, 'k--', 'LineWidth', 1.2); hold on;
plot(t_sine_out, theta_sine, 'LineWidth', 1.5);
grid on;
title('Closed-Loop Position Tracking for 10 Hz Sinusoid');
xlabel('Time [s]');
ylabel('\theta_{out} [rad]');
legend('Reference','Output','Location','southeast');

fprintf('\nPart II.b sinusoidal tracking summary\n');
fprintf('  Command frequency           = %.2f Hz\n', omega_sine/(2*pi));
fprintf('  Predicted steady-state gain = %.3f\n', mag_sine);
fprintf('  Predicted amplitude         = %.4f rad\n', predicted_amp);

% Bode comparison if experimental data are available in a MAT-file with
% variables freq_Hz, mag_dB, and phase_deg.
if exist('partII_bode_data.mat','file')
    data = load('partII_bode_data.mat');
    figure('Name','Part II.b - Bode Comparison','NumberTitle','off');
    bodemag(T_cl, {1, 1e4}); hold on;
    semilogx(2*pi*data.freq_Hz, data.mag_dB, 'o');
    grid on;
    title('Closed-Loop Bode Plot vs Experimental Data');
    legend('Model','Experimental','Location','southwest');
else
    warning(['Experimental Bode data file ''partII_bode_data.mat'' not found. ', ...
        'Skipping overlay plot.']);
end

%% Export results for quick access
results_part2 = struct();
results_part2.G_theta = G_theta;
results_part2.C_pd = C_pd;
results_part2.T_cl = T_cl;
results_part2.Kp = K_p;
results_part2.Kd = K_d;
results_part2.damping_ratio = damping_ratio;
results_part2.natural_frequency = natural_frequency;
results_part2.bandwidth = closed_loop_bandwidth;
results_part2.step_metrics = step_metrics;
results_part2.sinusoid_gain = mag_sine;
results_part2.sinusoid_phase_deg = phase_sine;
results_part2.predicted_sine_amplitude = predicted_amp;
assignin('base','midterm_part2_results',results_part2);

disp('Part II analysis complete.  Results available in midterm_part2_results.');
