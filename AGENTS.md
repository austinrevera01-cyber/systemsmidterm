# AGENTS Instructions

## Scope
These instructions apply to the entire `systemsmidterm` repository. Follow them for every file you modify here.

## Project context
This repo contains the Fall 2025 Mid-Semester Group Project for MAE 3140. Teams of four develop and validate models for a Maxon RE-30 (24 V) motor and design a steering controller that will later be exercised through the encrypted `run_Indy_car.p` plant.

Keep the following assignment deliverables in mind when adding or updating code, documentation, or analysis assets.

### Part I – Motor modeling (voltage → speed)
1. Build two motor models:
   - Include the armature inductance.
   - Neglect the armature inductance and clearly report the resulting effective inertia (\(J_{\text{eff}}\)) and viscous damping (\(b_{\text{eff}}\)).
2. Pull reference data for the 24 V Maxon RE-30, part #268214. Record any discrepancies you find between datasheet values and other sources.
3. Convert the manufacturer specifications into SI units. The expected table covers at least: back-emf constant \(K_b\), armature resistance \(R\), torque constant \(K_I\), stall torque, no-load speed, rotor mass moment of inertia, viscous damping coefficient \(b\), and the motor time constant.
4. For each model, determine the eigenvalues and corresponding time constants.
5. Simulate a 12 V step input. Plot both model responses (on the same axes) and explain the similarities and differences.
6. Compare the simulated time constants and steady-state speeds against the manufacturer specs. Justify when it is and is not acceptable to ignore inductance.
7. Re-derive the equations of motion for the motor with a gearbox of ratio \(N\) and compare your results to the provided MATLAB/Octave p-code.

### Part II – Steering motor position control
1. Design a controller for the RE-30 with the GP32 21:1 gearhead (part #166160) that achieves ≈5% overshoot with settling time < 0.025 s. Report the closed-loop bandwidth.
2. Simulate the controller on the Part I motor model for both constant and sinusoidal references. Compare simulated results with the predicted response and produce a Bode plot from experimental data.
3. Validate the controller with the provided `run_Indy_car.p` (consult `run_Indy_car_help.m`). The p-code accepts a steering motor voltage command (24 V max) and returns encoder angle after 1 ms along with GPS, yaw-rate gyro, waypoint, and lateral-error signals.
   - The steering encoder (part #110513) has 500 counts/rev; quadrature counting yields 4× resolution, and the simulation assumes a 12-bit counter (0–4095).
   - Use a 30° desired steering angle for baseline testing. A 1000° request is an extra-credit stress case that requires handling encoder wrap-around.
4. When driving the plant, only command steering voltage if you are evaluating the steering controller in isolation. Respect the gearhead’s assumption of negligible bearing losses and note the gearbox output rotates 15° for every 1° tire turn.

## Contribution guidelines
- Preserve compatibility with the encrypted p-code interface: the MATLAB signature is `[GPS, yaw_gyro, motor_counts, WP, lat_err] = run_Indy_car(volts, Vel, X0_values, WP_FILE)`.
- Clearly state assumptions (gear ratio \(N\), inductance inclusion, encoder count conversions, etc.) in code comments or documentation updates.
- When plotting comparisons, annotate which model (with vs. without inductance) or which reference trajectory each line represents.
- Favor SI units in new calculations; if you must use manufacturer units, document the conversion back to SI.
