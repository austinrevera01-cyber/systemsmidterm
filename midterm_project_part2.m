clear; clc; close all;

%% =================== MOTOR + GEAR PARAMETERS ===================
R  = 0.611;                 % Armature resistance [Ohm]
L  = 0.000119;              % Armature inductance [H]
Ki = 0.0259;                % Torque constant [N*m/A]
Kb = 0.025879;              % Back-EMF [V*s/rad]
Jm = 3.35e-6;               % Motor rotor inertia [kg*m^2]

% Gearbox (GP32 21:1)
N = 21;                     % Gear ratio (motor:output)
Jg_out = 0.8e-7;            % Gear inertia @ output [kg*m^2]
Jg_ref = Jg_out / N^2;      % Reflected gear inertia @ motor shaft
Jload = Jm + Jg_ref;        % Total inertia referred to motor side
J_out = Jload * N^2;        % Inertia at steering output

% Encoder metadata used in the run_Indy_car interface
CPR    = 500 * 4;           % Quadrature counts per motor revolution
MAXCNT = 4096;              % Encoder rollover count (12-bit counter)

%% =================== TRANSFER FUNCTION MODEL ===================
s = tf('s');
P_motor = Ki / ((Jload * s) * (L * s + R) + Ki * Kb);  % ω(s)/V(s)
P_theta = P_motor / s;                                % θ(s)/V(s)

%% =================== DESIGN SPECS ===================
zeta = 0.69;                % 5% overshoot
Ts   = 0.025;               % < 25 ms
wn   = 4.6 / (zeta * Ts);   % natural frequency heuristic
fprintf('Target ωn = %.1f rad/s\n', wn);

wbw = wn * sqrt(1 - 2 * zeta^2 + sqrt(2 + 4 * zeta^4));
fprintf('Closed-loop bandwidth ≈ %.1f rad/s (%.1f Hz)\n', wbw, wbw / (2 * pi));

%% =================== CONTROLLER DESIGN ===================
Kp = (wn^2 * J_out) / Ki;
Kd = (2 * zeta * wn * J_out - Kb * Ki / R) / Ki;
fprintf('Kp = %.2f, Kd = %.5f\n', Kp, Kd);

C = Kp + Kd * s;
T_closed = feedback(C * P_theta, 1);
info = stepinfo(T_closed);
fprintf('\nStep Response Metrics:\n');
fprintf('  Settling Time = %.4f s\n', info.SettlingTime);

%% =================== SINUSOIDAL TRACKING (VARIABLE REFERENCE) ===================
f_ref = 10;                      % 10 Hz reference signal (within bandwidth)
t = 0:1e-5:0.2;                  % fine time step
ref = 0.1 * sin(2 * pi * f_ref * t);     % sinusoidal reference
y = lsim(T_closed, ref, t);

%% =================== COMPARISON PLOTS SIDE BY SIDE ===================
figure('Name', 'Part II.b Responses', 'NumberTitle', 'off');
subplot(1, 2, 1);
step(T_closed);
title('Step Response (Constant Ref)');
ylabel('Position [rad]');
grid on;

subplot(1, 2, 2);
plot(t, y, 'b', 'LineWidth', 1.3); hold on;
plot(t, ref, 'r--', 'LineWidth', 1.2);
title(sprintf('Sinusoidal Tracking (%.1f Hz)', f_ref));
xlabel('Time [s]');
ylabel('Position [rad]');
legend('Output', 'Reference');
grid on;

%% =================== FREQUENCY RESPONSE ===================
figure('Name', 'Part II.b Closed-loop Bode', 'NumberTitle', 'off');
bode(T_closed);
grid on;
title('Bode Plot of Closed-loop Position System');

%% =================== Part II.c - Integral Controller Bench Test ===================
clear run_Indy_car; %#ok<CLRUN> reset persistent state inside the p-code

zeta_I = sqrt((log(0.05)^2) / (pi^2 + log(0.05)^2));
Ts_I   = 0.025;
wn_I   = 4 / (zeta_I * Ts_I);

Ki_I = (Jload * L / Ki) * wn_I^4;
fprintf('\nIntegral gain (continuous-time): Ki_I = %.2f\n', Ki_I);

dt    = 0.001;            % sample time enforced by run_Indy_car [s]
t_end = 10;               % simulation horizon [s]
steps = round(t_end / dt);
t_vec = (0:steps - 1).' * dt;

Vmax       = 24;          % actuator saturation [V]
counts2rad = 2 * pi / CPR;

theta_ref_deg = 30;
theta_ref     = deg2rad(theta_ref_deg);

Vel       = 0;                          % vehicle stationary for bench test
X0_values = [0 0 0 0 0];                % default initial conditions
WP_FILE   = 0;                          % no waypoint tracking
[~, ~, ~] = run_Indy_car(0, Vel, X0_values, WP_FILE); % clear internal state

acc_counts    = 0;
last_raw      = NaN;
motor_counts  = zeros(steps, 1);
theta_motor   = zeros(steps, 1);
theta_output  = zeros(steps, 1);
control_volts = zeros(steps, 1);

err_prev         = theta_ref;  % initial output is zero
integrator_state = 0;

for k = 1:steps
    tentative_state = integrator_state + err_prev * dt;
    u_unsat         = Ki_I * tentative_state;
    u               = max(min(u_unsat, Vmax), -Vmax);

    if u ~= u_unsat
        if (u > 0 && err_prev > 0) || (u < 0 && err_prev < 0)
            tentative_state = integrator_state;  % freeze integrator in windup
        end
    end

    integrator_state = tentative_state;

    control_volts(k) = u;

    [~, ~, counts] = run_Indy_car(u);
    raw = double(counts);

    if isnan(last_raw)
        acc_counts = raw;
    else
        delta = raw - last_raw;
        if delta >  MAXCNT / 2, delta = delta - MAXCNT; end
        if delta < -MAXCNT / 2, delta = delta + MAXCNT; end
        acc_counts = acc_counts + delta;
    end
    last_raw = raw;

    motor_counts(k) = acc_counts;
    theta_motor(k)  = acc_counts * counts2rad;
    theta_output(k) = theta_motor(k) / N;

    err_prev = theta_ref - theta_output(k);
end

theta_output_deg = rad2deg(theta_output);
tol              = 0.02 * abs(theta_ref);
settle_idx       = find(abs(theta_output - theta_ref) > tol, 1, 'last');
if isempty(settle_idx)
    Ts_measured = 0;
else
    Ts_measured = t_vec(min(settle_idx + 1, steps));
end

overshoot_pct = (max(theta_output) - theta_ref) / theta_ref * 100;
ss_error_deg  = rad2deg(theta_ref - mean(theta_output(round(0.9 * steps):end)));

fprintf('Measured overshoot  : %.2f%%%%\n', overshoot_pct);
fprintf('Measured Ts (2%%%%) : %.4f s\n', Ts_measured);
fprintf('Steady-state error  : %.4f deg\n', ss_error_deg);

figure('Name', 'Part II.c Steering Response', 'NumberTitle', 'off');
plot(t_vec, theta_output_deg, 'b', 'LineWidth', 1.5); hold on;
plot(t_vec, theta_ref_deg * ones(size(t_vec)), 'r--', 'LineWidth', 1.3);
grid on;
legend('run\_Indy\_car output', 'Reference', 'Location', 'southeast');
xlabel('Time [s]');
ylabel('Steering Angle [deg]');
title('Part II.c: Integral Controller vs. run\_Indy\_car.p');

figure('Name', 'Part II.c Control Effort', 'NumberTitle', 'off');
plot(t_vec, control_volts, 'LineWidth', 1.5);
yline([Vmax, -Vmax], 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Control Voltage [V]');
title('Integral Control Effort with Saturation Limits');

figure('Name', 'Part II.c Motor Angle', 'NumberTitle', 'off');
plot(t_vec, rad2deg(theta_motor), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Motor Angle [deg]');
title('Motor Shaft Angle (run\_Indy\_car)');

fclose('all');
clear run_Indy_car; %#ok<CLRUN> release p-code resources

%% =================== Part II.d - Vehicle State Simulation ===================
clear run_Indy_car; %#ok<CLRUN> reset persistent state inside the p-code

V_cmd       = 12;          % Constant motor command voltage [V]
vehicle_vel = 10;          % Vehicle forward speed [m/s]
dt_vehicle  = 0.001;       % Time step enforced by run_Indy_car [s]
t_final     = 5;           % Duration of the run [s]

encoder_cpr = CPR;         % Quadrature encoder counts per revolution
steer_ratio = 15;          % Overall steering ratio i = 15:1 (wheel:tire)
delta_max   = deg2rad(20); % Maximum physical steer angle at the tires [rad]

num_steps_vehicle = round(t_final / dt_vehicle);
time_vec_vehicle  = (0:num_steps_vehicle - 1)' * dt_vehicle;

X0      = [0 0 0 0 0];
WP_FILE = 0;
[~, ~, ~] = run_Indy_car(0, vehicle_vel, X0, WP_FILE); % one-time initialization

motor_counts_vehicle = zeros(num_steps_vehicle, 1);
delta_tire           = zeros(num_steps_vehicle, 1);
yaw_rate             = zeros(num_steps_vehicle, 1);
heading              = zeros(num_steps_vehicle, 1);

for k = 1:num_steps_vehicle
    [gps, yaw_k, counts_k] = run_Indy_car(V_cmd); %#ok<ASGLU>
    motor_counts_vehicle(k) = double(counts_k);
    yaw_rate(k)             = yaw_k;      % [rad/s]
    heading(k)              = gps(3);     % Heading psi [rad]
end

motor_angle_vehicle = unwrap(motor_counts_vehicle * (2 * pi / encoder_cpr));
delta_tire = motor_angle_vehicle / steer_ratio;
delta_tire = min(max(delta_tire, -delta_max), delta_max); % enforce physical limit

ma_window = ones(5, 1) / 5;
yaw_rate  = filter(ma_window, 1, yaw_rate);

figure('Name', 'Part D - Vehicle Output States', 'NumberTitle', 'off');
subplot(3, 1, 1);
plot(time_vec_vehicle, rad2deg(delta_tire), 'LineWidth', 1.5);
ylabel('\delta_{tire} [deg]');
grid on;
title('Steering Response for 12 V Command at 10 m/s');
xlim([0, 0.07]);

subplot(3, 1, 2);
plot(time_vec_vehicle, yaw_rate, 'LineWidth', 1.5);
ylabel('Yaw rate [rad/s]');
grid on;
xlim([0, 5]);

subplot(3, 1, 3);
plot(time_vec_vehicle, rad2deg(heading), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('\psi [deg]');
grid on;
xlim([0, 5]);

steady_idx_vehicle = round(0.9 * num_steps_vehicle):num_steps_vehicle;
delta_ss           = mean(delta_tire(steady_idx_vehicle));
yaw_ss             = mean(yaw_rate(steady_idx_vehicle));
psi_ss             = mean(heading(steady_idx_vehicle));

fprintf('\nSteady-state tire steer angle: %.2f deg\n', rad2deg(delta_ss));
fprintf('Steady-state yaw rate: %.4f rad/s\n', yaw_ss);
fprintf('Average heading over final 10%%: %.2f deg\n', rad2deg(psi_ss));

fclose('all');
clear run_Indy_car; %#ok<CLRUN> release p-code resources
