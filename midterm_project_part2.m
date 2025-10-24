clear; clc; close all;

%% =================== MOTOR + GEAR PARAMETERS ===================
R  = 0.611;                 % armature resistance [Ohm]
L  = 0.000119;              % armature inductance [H]
Ki = 0.0259;                % torque constant [N*m/A]
Kb = 0.025879;              % back-EMF [V*s/rad]
Jm = 3.35e-6;               % motor rotor inertia [kg*m^2]

% Gearbox (GP32 21:1)
N = 21;                     % gear ratio (motor:output)
Jg_out = 0.8e-7;            % gear inertia @ output [kg*m^2]
Jg_ref = Jg_out / N^2;      % reflected gear inertia @ motor shaft
Jload = Jm + Jg_ref;        % total inertia referred to motor side

%% =================== TRANSFER FUNCTION MODEL ===================
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

%% =================== FREQUENCY RESPONSE ===================
figure;
bode(T_closed);
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

%% =================== MOTOR + GEAR PARAMETERS ===================
R  = 0.611;                 % armature resistance [Ohm]
L  = 0.000119;              % armature inductance [H]
Ki = 0.0259;                % torque constant [N*m/A]
Kb = 0.025879;              % back-EMF [V*s/rad]
Jm = 3.35e-6;               % motor rotor inertia [kg*m^2]

% Gearbox (GP32 21:1)
N = 21;                     % gear ratio (motor:output)
Jg_out = 0.8e-7;            % gear inertia @ output [kg*m^2]
Jg_ref = Jg_out / N^2;      % reflected gear inertia @ motor shaft
Jload = Jm + Jg_ref;        % total inertia referred to motor side

%% =================== CONTROLLER GAIN (Part II.a) ===================
zeta = sqrt((log(0.05)^2) / (pi^2 + log(0.05)^2));  % 5%% overshoot target
Ts   = 0.025;               % settling time requirement [s]
wn   = 4 / (zeta * Ts);      % desired natural frequency

J_out = Jload * N^2;         % inertia reflected to the steering output side
% Match the closed-loop characteristic constant term (wn^4) of a squared
% second-order prototype to the motor + inductance model with integral
% control.  This yields an I-only gain that preserves the desired bandwidth
% target while eliminating steady-state error without proportional action.
Ki_I = (Jload * L / Ki) * wn^4;
fprintf('Integral gain (continuous-time): Ki_I = %.2f\n', Ki_I);

%% =================== DISCRETE-TIME SETUP ===================
dt      = 0.001;            % sample time enforced by run_Indy_car [s]
t_end   = 10;             % simulation horizon [s]
steps   = round(t_end / dt);
t       = (0:steps-1).' * dt;

Vmax    = 24;               % actuator saturation [V]
CPR     = 500 * 4;          % quadrature counts per motor revolution
MAXCNT  = 4096;             % encoder rollover count (12-bit)
counts2rad = 2*pi / CPR;    % conversion from counts to motor radians

% Reference trajectory (30 degree steering command at the tire)
theta_ref_deg = 30;
theta_ref     = deg2rad(theta_ref_deg);
%% =================== P-CODE INITIALIZATION ===================
Vel       = 0;                          % vehicle stationary for bench test
X0_values = [0 0 0 0 0];                % default initial conditions
WP_FILE   = 0;                          % no waypoint tracking
[~, ~, ~] = run_Indy_car(0, Vel, X0_values, WP_FILE); % clear internal state

%% =================== PREALLOCATION ===================
acc_counts    = 0;                      % unwrapped encoder counts
last_raw      = NaN;                    % previous raw count for rollover fix
motor_counts  = zeros(steps, 1);
theta_motor   = zeros(steps, 1);
theta_output  = zeros(steps, 1);
control_volts = zeros(steps, 1);
error_hist    = zeros(steps, 1);
integ_hist    = zeros(steps, 1);

% Initial error and integrator state
err_prev         = theta_ref;  % initial output is zero
integrator_state = 0;

%% =================== CLOSED-LOOP SIMULATION ===================
for k = 1:steps
    % I-only control in discrete time with simple anti-windup clamping
    tentative_state = integrator_state + err_prev * dt;
    u_unsat         = Ki_I * tentative_state;
    u               = u_unsat;
    if u_unsat > Vmax
        if err_prev > 0
            tentative_state = integrator_state;  % freeze integrator when saturating
        end
        u = Vmax;
    elseif u_unsat < -Vmax
        if err_prev < 0
            tentative_state = integrator_state;
        end
        u = -Vmax;
    end
    integrator_state = tentative_state;

    control_volts(k) = u;
    error_hist(k)    = err_prev;
    integ_hist(k)    = integrator_state;

    % Apply control to the p-code plant
    [~, ~, counts] = run_Indy_car(u);
    raw = double(counts);

    % Handle encoder rollover from the 12-bit counter
    if isnan(last_raw)
        acc_counts = raw;
    else
        delta = raw - last_raw;
        if delta >  MAXCNT/2, delta = delta - MAXCNT; end
        if delta < -MAXCNT/2, delta = delta + MAXCNT; end
        acc_counts = acc_counts + delta;
    end
    last_raw = raw;

    motor_counts(k) = acc_counts;
    theta_motor(k)  = acc_counts * counts2rad;
    theta_output(k) = theta_motor(k) / N;

    % Update error for next iteration
    err_prev       = theta_ref - theta_output(k);
end

%% =================== PERFORMANCE METRICS ===================
theta_output_deg = rad2deg(theta_output);
tol              = 0.02 * abs(theta_ref);
settle_idx       = find(abs(theta_output - theta_ref) > tol, 1, 'last');
if isempty(settle_idx)
    Ts_measured = 0;
else
    Ts_measured = t(min(settle_idx + 1, steps));
end

overshoot_pct = (max(theta_output) - theta_ref) / theta_ref * 100;
ss_error_rad   = theta_ref - mean(theta_output(round(0.9*steps):end));
ss_error_deg   = rad2deg(ss_error_rad);

fprintf('Measured overshoot  : %.2f%%%%\n', overshoot_pct);
fprintf('Measured Ts (2%%%%) : %.4f s\n', Ts_measured);
fprintf('Steady-state error  : %.4f deg\n', ss_error_deg);

%% =================== PLOTS ===================
figure('Name','Part II.c Steering Response','NumberTitle','off');
plot(t, theta_output_deg, 'b', 'LineWidth', 1.5); hold on;
plot(t, theta_ref_deg*ones(size(t)), 'r--', 'LineWidth', 1.3);
grid on;
legend('run_Indy_car output','Reference','Location','southeast');
xlabel('Time [s]');
ylabel('Steering Angle [deg]');
title('Part II.c: Integral Controller vs. run_Indy_car.p');

figure('Name','Part II.c Control Effort','NumberTitle','off');
plot(t, control_volts, 'LineWidth', 1.5);
yline([Vmax, -Vmax], 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Control Voltage [V]');
title('Integral Control Effort with Saturation Limits');

figure('Name','Part II.c Motor Angle','NumberTitle','off');
plot(t, rad2deg(theta_motor), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Motor Angle [deg]');
title('Motor Shaft Angle (run_Indy_car)');

%% =================== CLEANUP ===================
fclose('all');
clear run_Indy_car; %#ok<CLRUN> release p-code resources

%% partD_vehicle_state_simulation
% Simulate the Indy car steering response for Part D of the midterm project.
% A constant 12 V steering motor command is applied while the vehicle
% travels at 10 m/s.  The script records the steering angle at the tires,
% yaw rate, and heading and produces diagnostic plots.
%
% The encrypted ``run_Indy_car`` p-code enforces the maximum physical
% steering angle at the tires (± 20 deg).  We additionally convert the
% returned motor encoder counts to a tire steer angle by using the 15:1
% steering ratio provided in the project statement.

clearvars; close all; clc;
clear run_Indy_car; %#ok<CLRUN> reset persistent state inside the p-code

%% Configuration parameters for the simulation
V_cmd       = 12;          % Constant motor command voltage [V]
vehicle_vel = 10;          % Vehicle forward speed [m/s]
dt          = 0.001;       % Time step enforced by run_Indy_car [s]
t_final     = 5;           % Duration of the run [s]

encoder_cpr = 500 * 4;     % Quadrature encoder counts per revolution
steer_ratio = 15;          % Overall steering ratio i = 15:1 (wheel:tire)
delta_max   = deg2rad(20); % Maximum physical steer angle at the tires [rad]

num_steps = round(t_final / dt);
time_vec  = (0:num_steps-1)' * dt;

%% Initialize the p-code simulation with the specified velocity
X0      = [0 0 0 0 0];
WP_FILE = 0;
[~, ~, ~] = run_Indy_car(0, vehicle_vel, X0, WP_FILE); % one-time initialization

%% Storage for the outputs we care about
motor_counts = zeros(num_steps, 1);
delta_tire   = zeros(num_steps, 1);
yaw_rate     = zeros(num_steps, 1);
heading      = zeros(num_steps, 1);

%% Main simulation loop
for k = 1:num_steps
    [gps, yaw_k, counts_k] = run_Indy_car(V_cmd); %#ok<ASGLU> yaw_gyro == yaw rate
    motor_counts(k) = double(counts_k);
    yaw_rate(k)     = yaw_k;      % [rad/s]
    heading(k)      = gps(3);     % Heading psi [rad]
end

%% Convert encoder counts to motor angle and then to tire steer angle
% unwrap the encoder signal to remove rollover effects prior to conversion
motor_angle = unwrap(motor_counts * (2 * pi / encoder_cpr));
delta_tire = motor_angle / steer_ratio;
delta_tire = min(max(delta_tire, -delta_max), delta_max); % enforce physical limit

% Apply a small moving-average filter to the yaw-rate signal to reduce
% the discrete measurement noise returned by the p-code interface.  Using
% "filter" keeps the script compatible with both MATLAB and GNU Octave.
ma_window = ones(5,1) / 5;
yaw_rate = filter(ma_window, 1, yaw_rate);

%% Plot the requested vehicle output states
figure('Name', 'Part D - Vehicle Output States', 'NumberTitle', 'off');
subplot(3,1,1);
plot(time_vec, rad2deg(delta_tire), 'LineWidth', 1.5);
ylabel('\delta_{tire} [deg]');
grid on;
title('Steering Response for 12 V Command at 10 m/s');
xlim([0 , 0.07]);

subplot(3,1,2);
plot(time_vec, yaw_rate, 'LineWidth', 1.5);
ylabel('Yaw rate [rad/s]');
grid on;
xlim([0 , 5]);

subplot(3,1,3);
plot(time_vec, rad2deg(heading), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('\psi [deg]');
grid on;
xlim([0 , 5]);

%% Report steady-state values for reference
steady_idx = round(0.9 * num_steps):num_steps;
delta_ss = mean(delta_tire(steady_idx));
yaw_ss   = mean(yaw_rate(steady_idx));
psi_ss   = mean(heading(steady_idx));

fprintf('Steady-state tire steer angle: %.2f deg\n', rad2deg(delta_ss));
fprintf('Steady-state yaw rate: %.4f rad/s\n', yaw_ss);
fprintf('Average heading over final 10%%: %.2f deg\n', rad2deg(psi_ss));

%% Close any file handles opened inside the p-code
fclose('all');
