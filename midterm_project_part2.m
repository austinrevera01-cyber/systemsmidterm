clear; close all; clc;

params = motorGearParameters();

%% Part II.a/b: Continuous-time PD design and frequency analysis
pdResults = designPdController(params);

%% Part II.c: Integral controller evaluation with run_Indy_car.p
integralResults = evaluateIntegralController(params);

%% Part II.d: Vehicle-state simulation for constant 12 V command
vehicleResults = simulateVehicleStates(params);

assignin('base', 'midterm_part2_pd', pdResults);
assignin('base', 'midterm_part2_integral', integralResults);
assignin('base', 'midterm_part2_vehicle', vehicleResults);

fprintf('\nMidterm project Part II scripts finished. Results exported to workspace.\n');

%% ===== Local functions ==================================================
function params = motorGearParameters()
params.R  = 0.611;          % Armature resistance [Ohm]
params.L  = 0.000119;       % Armature inductance [H]
params.Ki = 0.0259;         % Torque constant [N*m/A]
params.Kb = 0.025879;       % Back-EMF constant [V*s/rad]
params.Jm = 3.35e-6;        % Rotor inertia [kg*m^2]

params.gearRatio        = 21;        % Motor:output ratio (GP32 21:1)
params.gearInertiaOut   = 0.8e-7;    % Gear inertia at output [kg*m^2]
params.encoderCounts    = 500 * 4;   % Quadrature counts per motor revolution
params.maxEncoderCount  = 4096;      % Encoder rollover count (12-bit)
params.steeringRatio    = 15;        % Motor:tire steering ratio
params.tireAngleLimit   = deg2rad(20); % Tire angle saturation [rad]

params.JgearRef = params.gearInertiaOut / params.gearRatio^2;
params.Jload    = params.Jm + params.JgearRef;     % Inertia at motor shaft
params.Joutput  = params.Jload * params.gearRatio^2; % Inertia reflected to output
end

function results = designPdController(params)
R = params.R;
L = params.L;
Ki = params.Ki;
Kb = params.Kb;
J = params.Jload;

s = tf('s');
P_motor = Ki / ((J * s) * (L * s + R) + Ki * Kb);
P_theta = P_motor / s;

fprintf('\nMotor angular position transfer function (theta/V):\n');
disp(P_theta);

% Design targets: 5%% overshoot, Ts < 25 ms
zeta = sqrt(log(0.05)^2 / (pi^2 + log(0.05)^2));
Ts   = 0.025;
wn   = 4 / (zeta * Ts);

fprintf('Target damping ratio  zeta = %.4f\n', zeta);
fprintf('Target natural freq. wn   = %.1f rad/s\n', wn);

wbw = wn * sqrt(1 - 2 * zeta^2 + sqrt(2 + 4 * zeta^4));
fprintf('Closed-loop bandwidth ≈ %.1f rad/s (%.1f Hz)\n', wbw, wbw / (2 * pi));

Kp = (wn^2 * params.Joutput) / Ki;
Kd = (2 * zeta * wn * params.Joutput - (Kb * Ki) / R) / Ki;
fprintf('PD gains: Kp = %.2f, Kd = %.5f\n', Kp, Kd);

C = Kp + Kd * s;
T_closed = feedback(C * P_theta, 1);
info = stepinfo(T_closed);
fprintf('Closed-loop overshoot  = %.2f%%%%\n', info.Overshoot);
fprintf('Closed-loop Ts (2%%%%) = %.4f s\n', info.SettlingTime);

figure('Name', 'Part II.a PD Step Response', 'NumberTitle', 'off');
step(T_closed);
title('Closed-loop Step Response (Position)');
ylabel('Angular Position [rad]');
grid on;

f_ref = 10;
t = (0:1e-5:0.2).';
ref = 0.1 * sin(2 * pi * f_ref * t);
y = lsim(T_closed, ref, t);

figure('Name', 'Part II.b Sinusoidal Tracking', 'NumberTitle', 'off');
plot(t, ref, 'r--', 'LineWidth', 1.3); hold on;
plot(t, y, 'b', 'LineWidth', 1.4);
legend('Reference', 'Output', 'Location', 'southeast');
title(sprintf('Sinusoidal Tracking (%.1f Hz)', f_ref));
xlabel('Time [s]');
ylabel('Angular Position [rad]');
grid on;

figure('Name', 'Part II.b Closed-loop Bode', 'NumberTitle', 'off');
bode(T_closed);
grid on;
title('Bode Plot of Closed-loop Position System');

results.transferFcn = P_theta;
results.controller  = C;
results.closedLoop  = T_closed;
results.stepInfo    = info;
results.zeta        = zeta;
results.wn          = wn;
results.bandwidth   = wbw;
results.Kp          = Kp;
results.Kd          = Kd;
results.sinusoid.t  = t;
results.sinusoid.ref = ref;
results.sinusoid.y   = y;
end

function results = evaluateIntegralController(params)
resetRunIndyCar();

zeta = sqrt(log(0.05)^2 / (pi^2 + log(0.05)^2));
Ts = 0.025;
wn = 4 / (zeta * Ts);
Ki_I = (params.Jload * params.L / params.Ki) * wn^4;
fprintf('\nPart II.c integral gain (continuous-time): Ki_I = %.2f\n', Ki_I);

results.KiIntegral = Ki_I;

if ~exist('run_Indy_car', 'file')
    warning('run_Indy_car.p not found on path. Skipping Part II.c simulation.');
    results.status = 'run_Indy_car.p unavailable';
    return;
end

% Discrete-time setup
results.status = 'Simulation executed';
dt = 0.001;
t_end = 10;
steps = round(t_end / dt);
t = (0:steps-1).' * dt;
Vmax = 24;
counts2rad = 2 * pi / params.encoderCounts;
theta_ref_deg = 30;
theta_ref = deg2rad(theta_ref_deg);

Vel = 0;
X0 = [0 0 0 0 0];
WP_FILE = 0;
run_Indy_car(0, Vel, X0, WP_FILE);

acc_counts = 0;
last_raw = NaN;
motor_counts = zeros(steps, 1);
theta_motor = zeros(steps, 1);
theta_output = zeros(steps, 1);
control_volts = zeros(steps, 1);
error_hist = zeros(steps, 1);
integ_hist = zeros(steps, 1);

err_prev = theta_ref;
integrator_state = 0;

for k = 1:steps
    tentative_state = integrator_state + err_prev * dt;
    u_unsat = Ki_I * tentative_state;
    u = min(max(u_unsat, -Vmax), Vmax);
    if (u ~= u_unsat) && sign(u_unsat) == sign(err_prev)
        tentative_state = integrator_state;
        u = sign(u_unsat) * Vmax;
    end
    integrator_state = tentative_state;

    control_volts(k) = u;
    error_hist(k) = err_prev;
    integ_hist(k) = integrator_state;

    [~, ~, counts] = run_Indy_car(u);
    raw = double(counts);

    if isnan(last_raw)
        acc_counts = raw;
    else
        delta = raw - last_raw;
        half_count = params.maxEncoderCount / 2;
        if delta > half_count
            delta = delta - params.maxEncoderCount;
        elseif delta < -half_count
            delta = delta + params.maxEncoderCount;
        end
        acc_counts = acc_counts + delta;
    end
    last_raw = raw;

    motor_counts(k) = acc_counts;
    theta_motor(k) = acc_counts * counts2rad;
    theta_output(k) = theta_motor(k) / params.gearRatio;

    err_prev = theta_ref - theta_output(k);
end

theta_output_deg = rad2deg(theta_output);
tol = 0.02 * abs(theta_ref);
settle_idx = find(abs(theta_output - theta_ref) > tol, 1, 'last');
if isempty(settle_idx)
    Ts_measured = 0;
else
    Ts_measured = t(min(settle_idx + 1, steps));
end

overshoot_pct = (max(theta_output) - theta_ref) / theta_ref * 100;
ss_error_rad = theta_ref - mean(theta_output(round(0.9 * steps):end));
ss_error_deg = rad2deg(ss_error_rad);

fprintf('Measured overshoot  : %.2f%%%%\n', overshoot_pct);
fprintf('Measured Ts (2%%%%) : %.4f s\n', Ts_measured);
fprintf('Steady-state error  : %.4f deg\n', ss_error_deg);

figure('Name', 'Part II.c Steering Response', 'NumberTitle', 'off');
plot(t, theta_output_deg, 'b', 'LineWidth', 1.5); hold on;
plot(t, theta_ref_deg * ones(size(t)), 'r--', 'LineWidth', 1.3);
legend('run\_Indy\_car output', 'Reference', 'Location', 'southeast');
xlabel('Time [s]');
ylabel('Steering Angle [deg]');
grid on;
title('Part II.c: Integral Controller vs. run\_Indy\_car.p');

figure('Name', 'Part II.c Control Effort', 'NumberTitle', 'off');
plot(t, control_volts, 'LineWidth', 1.5); hold on;
yline([Vmax, -Vmax], 'k--', 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Control Voltage [V]');
grid on;
title('Integral Control Effort with Saturation Limits');

figure('Name', 'Part II.c Motor Angle', 'NumberTitle', 'off');
plot(t, rad2deg(theta_motor), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Motor Angle [deg]');
grid on;
title('Motor Shaft Angle (run\_Indy\_car)');

results.time = t;
results.thetaOutputDeg = theta_output_deg;
results.thetaMotorDeg = rad2deg(theta_motor);
results.controlVolts = control_volts;
results.error = error_hist;
results.integrator = integ_hist;
results.overshootPercent = overshoot_pct;
results.settlingTime = Ts_measured;
results.ssErrorDeg = ss_error_deg;

fclose('all');
clear run_Indy_car;
end

function results = simulateVehicleStates(params)
resetRunIndyCar();

if ~exist('run_Indy_car', 'file')
    warning('run_Indy_car.p not found on path. Skipping Part II.d simulation.');
    results.status = 'run_Indy_car.p unavailable';
    return;
end

V_cmd = 12;
vehicle_vel = 10;
dt = 0.001;
t_final = 5;
num_steps = round(t_final / dt);
time_vec = (0:num_steps-1).' * dt;

X0 = [0 0 0 0 0];
WP_FILE = 0;
run_Indy_car(0, vehicle_vel, X0, WP_FILE);

motor_counts = zeros(num_steps, 1);
yaw_rate = zeros(num_steps, 1);
heading = zeros(num_steps, 1);

for k = 1:num_steps
    [gps, yaw_k, counts_k] = run_Indy_car(V_cmd); %#ok<ASGLU>
    motor_counts(k) = double(counts_k);
    yaw_rate(k) = yaw_k;
    heading(k) = gps(3);
end

motor_angle = unwrap(motor_counts * (2 * pi / params.encoderCounts));
delta_tire = motor_angle / params.steeringRatio;
delta_tire = min(max(delta_tire, -params.tireAngleLimit), params.tireAngleLimit);

yaw_rate = filter(ones(5,1) / 5, 1, yaw_rate);

figure('Name', 'Part II.d Vehicle Output States', 'NumberTitle', 'off');
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
xlim([0, 5]);

delta_ss = mean(delta_tire(round(0.9 * num_steps):end));
yaw_ss = mean(yaw_rate(round(0.9 * num_steps):end));
psi_ss = mean(heading(round(0.9 * num_steps):end));

fprintf('\nPart II.d steady-state tire steer angle: %.2f deg\n', rad2deg(delta_ss));
fprintf('Part II.d steady-state yaw rate: %.4f rad/s\n', yaw_ss);
fprintf('Part II.d average heading over final 10%%%%: %.2f deg\n', rad2deg(psi_ss));

results.status = 'Simulation executed';
results.time = time_vec;
results.deltaTireDeg = rad2deg(delta_tire);
results.yawRate = yaw_rate;
results.headingDeg = rad2deg(heading);
results.steadyState = struct( ...
    'delta_deg', rad2deg(delta_ss), ...
    'yaw_rate', yaw_ss, ...
    'heading_deg', rad2deg(psi_ss));

fclose('all');
clear run_Indy_car;
end

function resetRunIndyCar()
if exist('run_Indy_car', 'file')
    clear run_Indy_car;
end
end
