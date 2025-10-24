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
P_motor = Ki / ((Jload*s)*(L*s + R) + Ki*Kb);  % \omega(s)/V(s)
P_theta = P_motor / s;                         % \theta(s)/V(s)

fprintf('\nMotor angular position transfer function:\n');
disp(P_theta);

%% =================== DESIGN SPECS ===================
zeta = sqrt((log(0.05)^2) / (pi^2 + log(0.05)^2));  % 5%% overshoot
Ts   = 0.025;               % settling-time target (< 25 ms)
wn   = 4 / (zeta * Ts);      % natural frequency (second-order heuristic)
fprintf('Target damping ratio  zeta = %.4f\n', zeta);
fprintf('Target natural freq. wn   = %.1f rad/s\n', wn);

% Closed-loop bandwidth approximation for dominant second-order pair
wbw = wn * sqrt(1 - 2*zeta^2 + sqrt(2 + 4*zeta^4));
fprintf('Closed-loop bandwidth ≈ %.1f rad/s (%.1f Hz)\n', wbw, wbw/(2*pi));

%% =================== CONTROLLER DESIGN ===================
J_out = Jload * N^2;         % inertia reflected to output side
Kp = (wn^2 * J_out) / Ki;
Kd = (2*zeta*wn*J_out - (Kb*Ki)/R) / Ki;
fprintf('PD gains: Kp = %.2f, Kd = %.5f\n', Kp, Kd);

C = Kp + Kd*s;
T_closed = feedback(C * P_theta, 1);

info = stepinfo(T_closed);
fprintf('Closed-loop overshoot  = %.2f%%%%\n', info.Overshoot);
fprintf('Closed-loop Ts (2%%%%) = %.4f s\n', info.SettlingTime);

%% =================== STEP RESPONSE ===================
figure;
step(T_closed);
title('Closed-loop Step Response (Position)');
ylabel('Angular Position [rad]');
grid on;

%% =================== SINUSOIDAL TRACKING ===================
f_ref = 10;                          % 10 Hz reference signal (within bandwidth)
t = (0:1e-5:0.2).';                  % fine time step as column vector
ref = 0.1 * sin(2*pi*f_ref*t);       % sinusoidal reference (column vector)
y = lsim(T_closed, ref, t);

figure;
plot(t, ref, 'r--', 'LineWidth', 1.3); hold on;
plot(t, y, 'b', 'LineWidth', 1.4);
legend('Reference','Output');
title(sprintf('Sinusoidal Tracking (%.1f Hz)', f_ref));
xlabel('Time [s]');
ylabel('Angular Position [rad]');
grid on;

%% =================== FREQUENCY RESPONSE ===================
figure;
bode(T_closed);
grid on;
title('Bode Plot of Closed-loop Position System');
