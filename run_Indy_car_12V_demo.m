%% run_Indy_car_12V_demo
% Execute run_Indy_car.p with a 12 V command, record the motor encoder counts,
% convert them to radians, unwrap, differentiate, and plot the motor speed in
% rad/s derived from the p-code alone.

clearvars; close all; clc;
clear run_Indy_car; %#ok<CLRUN> Reset persistent state inside the p-code

%% Configuration
Vstep = 12;          % Input voltage [V]
dt    = 0.001;       % Sample time used by the p-code interface [s]
t_end = 0.5;         % Duration to simulate [s]
CPR   = 500 * 4;     % Quadrature counts per motor revolution
MAXCNT = 4096;       % Encoder rollover count

steps   = round(t_end / dt);
t_pcode = (0:steps-1)' * dt;

%% Locate and initialize the p-code
pfile = which('run_Indy_car.p');
if isempty(pfile)
    error('run_Indy_car.p not found on MATLAB path.');
end
fprintf('Found run_Indy_car.p at %s\n', pfile);

Vel       = 0;
X0_values = [0 0 0 0 0];
WP_FILE   = 0;
[~, ~, ~] = run_Indy_car(0, Vel, X0_values, WP_FILE); % one-time initialization

%% Run the voltage-only loop and accumulate encoder counts
acc_counts = 0;
last_raw   = NaN;
theta_counts = zeros(steps, 1);

for k = 1:steps
    [~, ~, counts] = run_Indy_car(Vstep);
    raw = double(counts);
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

%% Convert counts to motor angle (rad) and unwrap
theta_m = unwrap(theta_counts * (2 * pi / CPR));

%% Differentiate to obtain motor speed directly from the encoder
omega_m = zeros(steps, 1);
if steps >= 3
    omega_m(2:end-1) = (theta_m(3:end) - theta_m(1:end-2)) / (2 * dt);
    omega_m(1)       = (theta_m(2) - theta_m(1)) / dt;
    omega_m(end)     = (theta_m(end) - theta_m(end-1)) / dt;
end

%% Compute steady-state estimate from final 10 % of the record
tail_idx = max(1, round(0.9 * steps)):steps;
omega_m_ss = mean(omega_m(tail_idx));
rpm = @(w) w * 60 / (2 * pi);
fprintf('Steady-state (p-code): omega_m = %.3f rad/s (%.2f rpm)\n', omega_m_ss, rpm(omega_m_ss));

%% Plot the p-code derived motor speed only
figure('Name', 'run_Indy_car p-code motor speed');
plot(t_pcode, omega_m, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('\omega_{m} [rad/s]');
title('run\_Indy\_car p-code Motor Speed (12 V Step)');

%% Clean up file handles opened by run_Indy_car
fclose('all');
