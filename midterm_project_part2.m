%% Midterm Project Part II - Steering Motor Analysis and Simulation
% This script consolidates the controller design and validation tasks for
% Part II of the midterm project.  It reuses a common set of motor and gear
% parameters to:
%   * derive the closed-loop position dynamics and PD controller (Part II.a/b),
%   * evaluate sinusoidal tracking and frequency response of the design,
%   * exercise the discrete-time PI controller against the encrypted
%     ``run_Indy_car`` plant (Part II.c), and
%   * simulate the vehicle response to a constant steering command (Part II.d).
%
% The code is organized so that shared parameters are defined once and passed
% into helper functions.  Each section reports key metrics while generating the
% figures required for the write-up.  The Control System Toolbox is assumed to
% be available for transfer-function manipulation.

close all; clearvars; clc;

params = motorParameters();
gear   = gearboxParameters();
inertia = reflectedInertia(params, gear);
specs   = designSpecifications();

%% Part II.a/b - Continuous-Time PD Controller Design and Validation
pdResults = designPdPositionController(params, inertia, specs);

fprintf('\nPart II.a/b Summary:\n');
fprintf('  Damping ratio (zeta)        : %.4f\n', specs.zeta);
fprintf('  Natural frequency (wn)      : %.1f rad/s\n', specs.wn);
fprintf('  Closed-loop bandwidth       : %.1f rad/s (%.1f Hz)\n', ...
    specs.bandwidth_rad, specs.bandwidth_rad/(2*pi));
fprintf('  PD gains                    : Kp = %.2f, Kd = %.5f\n', ...
    pdResults.Kp, pdResults.Kd);
fprintf('  Step response overshoot     : %.2f%%%%\n', pdResults.stepInfo.Overshoot);
fprintf('  Step response Ts (2%%%% band): %.4f s\n', pdResults.stepInfo.SettlingTime);

%% Part II.c - PI Controller Bench Test with run_Indy_car
piResults = runPiController(params, gear, inertia, specs);

fprintf('\nPart II.c Summary:\n');
fprintf('  PI gains (continuous)       : Kp = %.2f, Ki = %.2f\n', ...
    piResults.Kp_PI, piResults.Ki_PI);
fprintf('  Measured overshoot          : %.2f%%%%\n', piResults.overshoot_pct);
fprintf('  Measured Ts (2%%%% band)     : %.4f s\n', piResults.settling_time);
fprintf('  Steady-state error          : %.4f deg\n', piResults.ss_error_deg);

%% Part II.d - Vehicle Output State Simulation
vehicleResults = runVehicleStateSimulation(gear);

fprintf('\nPart II.d Summary:\n');
fprintf('  Steady-state tire angle     : %.2f deg\n', vehicleResults.delta_ss_deg);
fprintf('  Steady-state yaw rate       : %.4f rad/s\n', vehicleResults.yaw_ss);
fprintf('  Avg. heading (final 10%%%%)  : %.2f deg\n', vehicleResults.psi_ss_deg);

%% ------------------------------------------------------------------------
%% Local helper functions
function params = motorParameters()
%MOTORPARAMETERS Returns electrical and mechanical motor constants.
params.R  = 0.611;        % Armature resistance [Ohm]
params.L  = 0.000119;     % Armature inductance [H]
params.Ki = 0.0259;       % Torque constant [N*m/A]
params.Kb = 0.025879;     % Back-EMF constant [V*s/rad]
params.Jm = 3.35e-6;      % Motor rotor inertia [kg*m^2]
end

function gear = gearboxParameters()
%GEARBOXPARAMETERS Returns gearbox geometry and limits.
gear.N        = 21;        % Gear ratio (motor:output)
gear.Jg_out   = 0.8e-7;    % Gear inertia at output [kg*m^2]
gear.encoder_cpr = 500 * 4; % Quadrature counts per motor revolution
gear.steer_ratio = 15;     % Overall steering ratio (wheel:tire)
gear.delta_max  = deg2rad(20); % Physical tire angle limit [rad]
end

function inertia = reflectedInertia(params, gear)
%REFLECTEDINERTIA Computes inertia at the motor and output shafts.
Jg_ref = gear.Jg_out / gear.N^2; % Reflected gear inertia at motor shaft
inertia.motor  = params.Jm + Jg_ref;  % Total inertia referred to motor side
inertia.output = inertia.motor * gear.N^2; % Equivalent inertia at output
end

function specs = designSpecifications()
%DESIGNSPECIFICATIONS Computes desired damping and bandwidth targets.
desired_overshoot = 0.05; % 5% overshoot target
specs.zeta = sqrt((log(desired_overshoot)^2) / ...
                  (pi^2 + log(desired_overshoot)^2));
specs.Ts = 0.025;         % Settling time requirement (< 25 ms)
specs.wn = 4 / (specs.zeta * specs.Ts); % 2% settling-time heuristic
specs.bandwidth_rad = specs.wn * sqrt(1 - 2*specs.zeta^2 + ...
    sqrt(2 + 4*specs.zeta^4));
end

function results = designPdPositionController(params, inertia, specs)
%DESIGNPDPOSITIONCONTROLLER Designs and evaluates the PD controller.
s = tf('s');
P_motor = Ki / ((Jload*s)*(L*s + R) + Ki*Kb);  % ω(s)/V(s)
P_theta = P_motor / s;  % θ(s)/V(s)

%% =================== DESIGN SPECS ===================
zeta = 0.69;                % 5% overshoot
Ts   = 0.025;               % < 25 ms
wn   = 4.6 / (zeta*Ts);       % natural frequency
fprintf('Target ωn = %.1f rad/s\n', wn);

% Closed-loop bandwidth
wbw = wn * sqrt(1 - 2*zeta^2 + sqrt(2 + 4*zeta^4));
fprintf('Closed-loop bandwidth ≈ %.1f rad/s (%.1f Hz)\n', wbw, wbw/(2*pi));

%% =================== CONTROLLER DESIGN ===================
J_out = Jload * N^2;  % inertia at output
Kp = (wn^2 * J_out) / Ki;
Kd = (2*zeta*wn*J_out - Kb*Ki/R) / Ki;
fprintf('Kp = %.2f, Kd = %.5f\n', Kp, Kd);

C = Kp + Kd*s;
T_closed = feedback(C*P_theta, 1);
info = stepinfo(T_closed);
fprintf('\nStep Response Metrics:\n');
fprintf('  Settling Time = %.4f s\n', info.SettlingTime);

%% =================== SINUSOIDAL TRACKING (VARIABLE REFERENCE) ===================
f_ref = 10;                      % 10 Hz reference signal (within bandwidth)
t = 0:1e-5:0.2;                  % fine time step
ref = 0.1*sin(2*pi*f_ref*t);     % sinusoidal reference

y = lsim(T_closed, ref, t);

%% =================== COMPARISON PLOTS SIDE BY SIDE ===================
figure;
subplot(1,2,1);
step(T_closed);
title('Step Response (Constant Ref)');
ylabel('Position [rad]');
grid on;

subplot(1,2,2);
plot(t, y, 'b', 'LineWidth', 1.3); hold on;
plot(t, ref, 'r--', 'LineWidth', 1.2);
title(sprintf('Sinusoidal Tracking (%.1f Hz)', f_ref));
xlabel('Time [s]');
ylabel('Position [rad]');
legend('Output','Reference');
grid on;

% Frequency response
figure('Name','Part II.b - Closed-loop Bode','NumberTitle','off');
bode(results.T_closed);
grid on;
end

function results = runPiController(params, gear, inertia, specs)
%RUNPICONTROLLER Discrete PI controller on run_Indy_car.

% Continuous-time gains obtained by matching the s and constant terms of
% the desired (s^2 + 2*zeta*wn*s + wn^2)^2 characteristic polynomial to the
% motor + inductance dynamics referenced to the steering output.  The extra
% factor of gear.N accounts for commanding the tire angle instead of the
% motor shaft angle.
gain_scale = inertia.motor * params.L * gear.N / params.Ki;
Kp_cont = gain_scale * (4 * specs.zeta * specs.wn^3);
Ki_cont = gain_scale * (specs.wn^4);

% Discrete-time setup enforced by the p-code interface
resetRunIndyCar();
dt    = 0.001;        % Sample time [s]
t_end = 10;           % Simulation horizon [s]
steps = round(t_end / dt);
t     = (0:steps-1).' * dt;

Vmax = 24;            % Actuator saturation [V]
counts2rad = 2 * pi / gear.encoder_cpr;

% Reference trajectory (30 degree steering command at the tire)
theta_ref = deg2rad(30);

% Initialize p-code with bench-test conditions
vehicle_speed = 0;
X0 = zeros(1,5);
WP_FILE = 0;
[~, ~, ~] = run_Indy_car(0, vehicle_speed, X0, WP_FILE);

% Preallocate logs
acc_counts    = 0;
last_raw      = NaN;
motor_counts  = zeros(steps, 1);
theta_motor   = zeros(steps, 1);
theta_output  = zeros(steps, 1);
control_volts = zeros(steps, 1);
prop_hist     = zeros(steps, 1);
error_hist    = zeros(steps, 1);
integ_hist    = zeros(steps, 1);

integrator_state   = 0;
theta_output_prev  = 0;

for k = 1:steps
    err_k = theta_ref - theta_output_prev;
    tentative_state = integrator_state + err_k * dt;
    prop_term = Kp_cont * err_k;
    u_unsat = prop_term + Ki_cont * tentative_state;
    u = saturate(u_unsat, Vmax);

    sat_active = abs(u_unsat - u) > 1e-9;
    if ~sat_active || sign(err_k) ~= sign(u)
        integrator_state = tentative_state;
    end

    control_volts(k) = u;
    prop_hist(k)     = prop_term;
    error_hist(k)    = err_k;
    integ_hist(k)    = integrator_state;

    % Apply control to the p-code plant
    [~, ~, counts] = run_Indy_car(u);
    raw = double(counts);

    % Handle encoder rollover from the 12-bit counter
    if isnan(last_raw)
        acc_counts = raw;
    else
        delta = raw - last_raw;
        if delta >  4096/2, delta = delta - 4096; end
        if delta < -4096/2, delta = delta + 4096; end
        acc_counts = acc_counts + delta;
    end
    last_raw = raw;

    motor_counts(k) = acc_counts;
    theta_motor(k)  = acc_counts * counts2rad;
    theta_output(k) = theta_motor(k) / gear.N;
    theta_output_prev = theta_output(k);
end

theta_output_deg = rad2deg(theta_output);

% Settling time (2% band)
tol = 0.02 * abs(theta_ref);
settle_idx = find(abs(theta_output - theta_ref) > tol, 1, 'last');
if isempty(settle_idx)
    settling_time = 0;
else
    settling_time = t(min(settle_idx + 1, steps));
end

results.Kp_PI = Kp_cont;
results.Ki_PI = Ki_cont;
results.overshoot_pct = (max(theta_output) - theta_ref) / theta_ref * 100;
results.settling_time = settling_time;
results.ss_error_deg  = rad2deg(theta_ref - mean(theta_output(round(0.9*steps):end)));
results.time          = t;
results.theta_output_deg = theta_output_deg;
results.control_volts = control_volts;
results.prop_history  = prop_hist;
results.error_history = error_hist;
results.integrator_state = integ_hist;

% Plot response and diagnostics
figure('Name','Part II.c - Steering Response','NumberTitle','off');
plot(t, theta_output_deg, 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(theta_ref) * ones(size(t)), 'r--', 'LineWidth', 1.3);
legend('run\_Indy\_car output','Reference','Location','southeast');
xlabel('Time [s]');
ylabel('Steering Angle [deg]');
grid on;
title('Part II.c: PI Controller Tracking');

figure('Name','Part II.c - Control Effort','NumberTitle','off');
plot(t, control_volts, 'LineWidth', 1.5); hold on;
yline([Vmax, -Vmax], 'k--', 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Control Voltage [V]');
grid on;
title('PI Control Effort with Saturation Limits');

figure('Name','Part II.c - Motor Angle','NumberTitle','off');
plot(t, rad2deg(theta_motor), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Motor Angle [deg]');
grid on;
title('Motor Shaft Angle (run\_Indy\_car)');

fclose('all');
clear run_Indy_car;
end

function results = runVehicleStateSimulation(gear)
%RUNVEHICLESTATESIMULATION Simulates vehicle response at 10 m/s.
resetRunIndyCar();

V_cmd       = 12;          % Constant motor command [V]
vehicle_vel = 10;          % Forward speed [m/s]
dt          = 0.001;       % Time step [s]
t_final     = 5;           % Simulation length [s]

num_steps = round(t_final / dt);
time_vec  = (0:num_steps-1)' * dt;

X0 = zeros(1,5);
WP_FILE = 0;
[~, ~, ~] = run_Indy_car(0, vehicle_vel, X0, WP_FILE);

motor_counts = zeros(num_steps, 1);
yaw_rate     = zeros(num_steps, 1);
heading      = zeros(num_steps, 1);

for k = 1:num_steps
    [gps, yaw_k, counts_k] = run_Indy_car(V_cmd); %#ok<ASGLU>
    motor_counts(k) = double(counts_k);
    yaw_rate(k)     = yaw_k;
    heading(k)      = gps(3);
end

motor_angle = unwrap(motor_counts * (2 * pi / gear.encoder_cpr));
delta_tire = motor_angle / gear.steer_ratio;
delta_tire = min(max(delta_tire, -gear.delta_max), gear.delta_max);

yaw_rate = filter(ones(5,1)/5, 1, yaw_rate);

figure('Name','Part II.d - Vehicle Output States','NumberTitle','off');
subplot(3,1,1);
plot(time_vec, rad2deg(delta_tire), 'LineWidth', 1.5);
ylabel('\delta_{tire} [deg]');
grid on;
title('Steering Response for 12 V Command at 10 m/s');
xlim([0, 0.07]);

subplot(3,1,2);
plot(time_vec, yaw_rate, 'LineWidth', 1.5);
ylabel('Yaw rate [rad/s]');
grid on;
xlim([0, 5]);

subplot(3,1,3);
plot(time_vec, rad2deg(heading), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('\psi [deg]');
grid on;
title('Bode Plot of Closed-loop Position System');

%% Midterm Project Part II.c - Controller test with run_Indy_car.p
% This script exercises an integral controller (I-only) designed from the
% Part II specifications on the encrypted run_Indy_car.p plant.  The goal is
% to track a 30 degree
% steering command while respecting the p-code interface (1 ms sample
% period, encoder counts at the motor shaft, and 24 V input limit).
%
% The script re-computes the continuous-time design targets from
% midterm_project_part2.m so it can be run independently, then executes a
% discrete-time loop that wraps the p-code function.  The encoder counts are
% unwrapped to radians, converted to the steering angle through the gear
% ratio, and fed back to the integral controller.  Key performance metrics
% are reported alongside diagnostic plots of the response and control effort.

clearvars; close all; clc;
clear run_Indy_car; %#ok<CLRUN> reset persistent state inside the p-code

params = motorGearParameters();

partIIAB_ControllerDesign(params);
partIIC_IntegralBenchTest(params);
partIID_VehicleStateSimulation(params);

function params = motorGearParameters()
params.R       = 0.611;        % Armature resistance [Ohm]
params.L       = 0.000119;     % Armature inductance [H]
params.Ki      = 0.0259;       % Torque constant [N*m/A]
params.Kb      = 0.025879;     % Back-EMF constant [V*s/rad]
params.Jm      = 3.35e-6;      % Motor rotor inertia [kg*m^2]
params.N       = 21;           % Gear ratio (motor:output)
params.Jg_out  = 0.8e-7;       % Gear inertia at output [kg*m^2]
params.Jg_ref  = params.Jg_out / params.N^2;
params.Jload   = params.Jm + params.Jg_ref;  % Total inertia at motor shaft
params.J_out   = params.Jload * params.N^2;  % Inertia at steering output
params.CPR     = 500 * 4;      % Encoder counts per motor revolution
params.MAXCNT  = 4096;         % Encoder rollover (12-bit counter)
end

function partIIAB_ControllerDesign(params)
s = tf('s');

P_motor = params.Ki / ((params.Jload * s) * (params.L * s + params.R) + params.Ki * params.Kb);
P_theta = P_motor / s;

zeta = 0.69;               % 5% overshoot target
Ts   = 0.025;              % < 25 ms settling time
wn   = 4.6 / (zeta * Ts);  % Natural frequency heuristic

fprintf('Target ωn = %.1f rad/s\n', wn);
wbw = wn * sqrt(1 - 2 * zeta^2 + sqrt(2 + 4 * zeta^4));
fprintf('Closed-loop bandwidth ≈ %.1f rad/s (%.1f Hz)\n', wbw, wbw / (2 * pi));

Kp = (wn^2 * params.J_out) / params.Ki;
Kd = (2 * zeta * wn * params.J_out - params.Kb * params.Ki / params.R) / params.Ki;

fprintf('Kp = %.2f, Kd = %.5f\n', Kp, Kd);

C = Kp + Kd * s;
T_closed = feedback(C * P_theta, 1);

info = stepinfo(T_closed);
fprintf('\nStep Response Metrics:\n');
fprintf('  Settling Time = %.4f s\n', info.SettlingTime);

f_ref = 10;                   % 10 Hz reference within bandwidth
t     = 0:1e-5:0.2;
ref   = 0.1 * sin(2 * pi * f_ref * t);
y     = lsim(T_closed, ref, t);

figure('Name', 'Part II.b Responses', 'NumberTitle', 'off');
subplot(1, 2, 1);
step(T_closed);
title('Step Response (Constant Ref)');
ylabel('Position [rad]');
grid on;
legend('run_Indy_car output','Reference','Location','southeast');
xlabel('Time [s]');
ylabel('Steering Angle [deg]');
title('Part II.c: Integral Controller vs. run_Indy_car.p');

subplot(1, 2, 2);
plot(t, y, 'b', 'LineWidth', 1.3); hold on;
plot(t, ref, 'r--', 'LineWidth', 1.2);
title(sprintf('Sinusoidal Tracking (%.1f Hz)', f_ref));
xlabel('Time [s]');
ylabel('Position [rad]');
legend('Output', 'Reference');
grid on;
title('Steering Response for 12 V Command at 10 m/s');
xlim([0 , 0.07]);

figure('Name', 'Part II.b Closed-loop Bode', 'NumberTitle', 'off');
bode(T_closed);
grid on;
title('Bode Plot of Closed-loop Position System');
end

function partIIC_IntegralBenchTest(params)
clear run_Indy_car; %#ok<CLRUN> reset persistent state inside the p-code

zeta = sqrt((log(0.05)^2) / (pi^2 + log(0.05)^2));
Ts   = 0.025;
wn   = 4 / (zeta * Ts);

Ki_I = (params.Jload * params.L / params.Ki) * wn^4;
fprintf('\nIntegral gain (continuous-time): Ki_I = %.2f\n', Ki_I);

dt    = 0.001;
t_end = 10;
steps = round(t_end / dt);
t     = (0:steps - 1).' * dt;

Vmax       = 24;
counts2rad = 2 * pi / params.CPR;

theta_ref_deg = 30;
theta_ref     = deg2rad(theta_ref_deg);

Vel       = 0;
X0_values = [0 0 0 0 0];
WP_FILE   = 0;
[~, ~, ~] = run_Indy_car(0, Vel, X0_values, WP_FILE);

acc_counts    = 0;
last_raw      = NaN;
motor_counts  = zeros(steps, 1);
theta_motor   = zeros(steps, 1);
theta_output  = zeros(steps, 1);
control_volts = zeros(steps, 1);
error_hist    = zeros(steps, 1);
integ_hist    = zeros(steps, 1);

err_prev         = theta_ref;
integrator_state = 0;

for k = 1:steps
    tentative_state = integrator_state + err_prev * dt;
    u_unsat         = Ki_I * tentative_state;
    u               = max(min(u_unsat, Vmax), -Vmax);

    if (u ~= u_unsat)
        if (u > 0 && err_prev > 0) || (u < 0 && err_prev < 0)
            tentative_state = integrator_state;
        end
    end

    integrator_state = tentative_state;

    control_volts(k) = u;
    error_hist(k)    = err_prev;
    integ_hist(k)    = integrator_state;

    [~, ~, counts] = run_Indy_car(u);
    raw = double(counts);

    if isnan(last_raw)
        acc_counts = raw;
    else
        delta = raw - last_raw;
        if delta >  params.MAXCNT / 2, delta = delta - params.MAXCNT; end
        if delta < -params.MAXCNT / 2, delta = delta + params.MAXCNT; end
        acc_counts = acc_counts + delta;
    end
    last_raw = raw;

    motor_counts(k) = acc_counts;
    theta_motor(k)  = acc_counts * counts2rad;
    theta_output(k) = theta_motor(k) / params.N;

    err_prev = theta_ref - theta_output(k);
end

theta_output_deg = rad2deg(theta_output);
tol              = 0.02 * abs(theta_ref);
settle_idx       = find(abs(theta_output - theta_ref) > tol, 1, 'last');
if isempty(settle_idx)
    Ts_measured = 0;
else
    Ts_measured = t(min(settle_idx + 1, steps));
end

overshoot_pct = (max(theta_output) - theta_ref) / theta_ref * 100;
ss_error_deg  = rad2deg(theta_ref - mean(theta_output(round(0.9 * steps):end)));

fprintf('Measured overshoot  : %.2f%%%%\n', overshoot_pct);
fprintf('Measured Ts (2%%%%) : %.4f s\n', Ts_measured);
fprintf('Steady-state error  : %.4f deg\n', ss_error_deg);

figure('Name', 'Part II.c Steering Response', 'NumberTitle', 'off');
plot(t, theta_output_deg, 'b', 'LineWidth', 1.5); hold on;
plot(t, theta_ref_deg * ones(size(t)), 'r--', 'LineWidth', 1.3);
grid on;
legend('run\_Indy\_car output', 'Reference', 'Location', 'southeast');
xlabel('Time [s]');
ylabel('Steering Angle [deg]');
title('Part II.c: Integral Controller vs. run\_Indy\_car.p');

figure('Name', 'Part II.c Control Effort', 'NumberTitle', 'off');
plot(t, control_volts, 'LineWidth', 1.5);
yline([Vmax, -Vmax], 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Control Voltage [V]');
title('Integral Control Effort with Saturation Limits');

figure('Name', 'Part II.c Motor Angle', 'NumberTitle', 'off');
plot(t, rad2deg(theta_motor), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Motor Angle [deg]');
title('Motor Shaft Angle (run\_Indy\_car)');

fclose('all');
clear run_Indy_car; %#ok<CLRUN> release p-code resources
end

function partIID_VehicleStateSimulation(params)
clear run_Indy_car; %#ok<CLRUN> reset persistent state inside the p-code

V_cmd       = 12;
vehicle_vel = 10;
dt          = 0.001;
t_final     = 5;

encoder_cpr = params.CPR;
steer_ratio = 15;
delta_max   = deg2rad(20);

num_steps = round(t_final / dt);
time_vec  = (0:num_steps - 1)' * dt;

X0      = [0 0 0 0 0];
WP_FILE = 0;
[~, ~, ~] = run_Indy_car(0, vehicle_vel, X0, WP_FILE);

motor_counts = zeros(num_steps, 1);
delta_tire   = zeros(num_steps, 1);
yaw_rate     = zeros(num_steps, 1);
heading      = zeros(num_steps, 1);

for k = 1:num_steps
    [gps, yaw_k, counts_k] = run_Indy_car(V_cmd); %#ok<ASGLU>
    motor_counts(k) = double(counts_k);
    yaw_rate(k)     = yaw_k;
    heading(k)      = gps(3);
end

motor_angle = unwrap(motor_counts * (2 * pi / encoder_cpr));
delta_tire = min(max(motor_angle / steer_ratio, -delta_max), delta_max);

ma_window = ones(5, 1) / 5;
yaw_rate  = filter(ma_window, 1, yaw_rate);

figure('Name', 'Part D - Vehicle Output States', 'NumberTitle', 'off');
subplot(3, 1, 1);
plot(time_vec, rad2deg(delta_tire), 'LineWidth', 1.5);
ylabel('\delta_{tire} [deg]');
grid on;
title('Steering Response for 12 V Command at 10 m/s');
xlim([0, 0.07]);

subplot(3, 1, 2);
plot(time_vec, yaw_rate, 'LineWidth', 1.5);
ylabel('Yaw rate [rad/s]');
grid on;
xlim([0, 5]);

subplot(3, 1, 3);
plot(time_vec, rad2deg(heading), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('\psi [deg]');
grid on;
xlim([0, 5]);

steady_idx = round(0.9 * num_steps):num_steps;
delta_ss   = mean(delta_tire(steady_idx));
yaw_ss     = mean(yaw_rate(steady_idx));
psi_ss     = mean(heading(steady_idx));

fprintf('\nSteady-state tire steer angle: %.2f deg\n', rad2deg(delta_ss));
fprintf('Steady-state yaw rate: %.4f rad/s\n', yaw_ss);
fprintf('Average heading over final 10%%: %.2f deg\n', rad2deg(psi_ss));

fclose('all');
clear run_Indy_car; %#ok<CLRUN> release p-code resources
end
