%% Quaternion Attitude Control with Reaction Wheels

clear;
clc;
close all;

%% Load parameters
init_attitude_3axis;
init_reaction_wheels;

%% Initial and commanded attitudes
combined_attitude0 = deg2rad([30; -20; 15]);

initial_quaternion = ...
    euler_to_quaternion(combined_attitude0);

command_quaternion = ...
    euler_to_quaternion(attitude_cmd);

initial_wheel_speed = [0; 0; 0];

%% Ten-state initial condition
initial_state = [
    initial_quaternion;
    omega0;
    initial_wheel_speed
];

%% Simulation settings
time = (0:0.01:t_final).';

solver_options = odeset( ...
    'RelTol', 1e-10, ...
    'AbsTol', 1e-12);

%% Run simulation
[t, state] = ode45( ...
    @(time, state) closed_loop_dynamics_reaction_wheels( ...
        time, ...
        state, ...
        command_quaternion, ...
        Kp, ...
        Kd, ...
        I, ...
        wheel_inertia, ...
        wheel_torque_max, ...
        wheel_speed_max), ...
    time, ...
    initial_state, ...
    solver_options);

%% Extract states
quaternion_raw = state(:,1:4);
omega = state(:,5:7);
wheel_speed = state(:,8:10);

quaternion_norm = sqrt(sum(quaternion_raw.^2, 2));

maximum_norm_error = ...
    max(abs(quaternion_norm - 1));

quaternion_history = ...
    quaternion_raw ./ quaternion_norm;

number_of_samples = length(t);

attitude_deg = zeros(number_of_samples, 3);
spacecraft_torque = zeros(number_of_samples, 3);
wheel_motor_torque = zeros(number_of_samples, 3);
speed_limited = false(number_of_samples, 3);
error_angle_deg = zeros(number_of_samples, 1);

%% Reconstruct outputs
for k = 1:number_of_samples
    current_quaternion = quaternion_history(k,:).';

    attitude_deg(k,:) = rad2deg( ...
        quaternion_to_euler(current_quaternion)).';

    [~, requested_torque, error_quaternion] = ...
        pd_controller_quaternion( ...
            command_quaternion, ...
            current_quaternion, ...
            omega(k,:).', ...
            Kp, ...
            Kd, ...
            wheel_torque_max);

    [spacecraft_torque_k, ~, wheel_torque_k, ...
        speed_limited_k] = reaction_wheel_actuator( ...
            requested_torque, ...
            wheel_speed(k,:).', ...
            wheel_inertia, ...
            wheel_torque_max, ...
            wheel_speed_max);

    spacecraft_torque(k,:) = spacecraft_torque_k.';
    wheel_motor_torque(k,:) = wheel_torque_k.';
    speed_limited(k,:) = speed_limited_k.';

    error_scalar = min(1, max(-1, ...
        abs(error_quaternion(1))));

    error_angle_deg(k) = ...
        rad2deg(2 * acos(error_scalar));
end

%% Derived quantities
omega_deg = rad2deg(omega);

wheel_speed_rpm = ...
    wheel_speed * 60 / (2*pi);

wheel_momentum = ...
    wheel_speed .* wheel_inertia.';

spacecraft_momentum = omega * I;

total_momentum_body = ...
    spacecraft_momentum + wheel_momentum;

total_momentum_magnitude = ...
    vecnorm(total_momentum_body, 2, 2);

%% Performance measurements
final_attitude_deg = attitude_deg(end,:);
final_error_angle_deg = error_angle_deg(end);

max_body_rate_deg = max(abs(omega_deg), [], 1);
max_spacecraft_torque = ...
    max(abs(spacecraft_torque), [], 1);

max_wheel_speed_rpm = ...
    max(abs(wheel_speed_rpm), [], 1);

final_wheel_speed_rpm = wheel_speed_rpm(end,:);

max_wheel_momentum = ...
    max(abs(wheel_momentum), [], 1);

speed_limit_activated = any(speed_limited, 1);

tolerance_deg = 0.5;
settling_time = NaN;

for k = 1:number_of_samples
    if all(error_angle_deg(k:end) <= tolerance_deg)
        settling_time = t(k);
        break;
    end
end

%% Display results
fprintf('\nFinal attitude [deg]:\n');
fprintf('Roll:  %.6f\n', final_attitude_deg(1));
fprintf('Pitch: %.6f\n', final_attitude_deg(2));
fprintf('Yaw:   %.6f\n', final_attitude_deg(3));

fprintf('\nFinal quaternion error angle: %.6f deg\n', ...
    final_error_angle_deg);

fprintf('\nMaximum body rate [deg/s]:\n');
fprintf('p: %.6f\n', max_body_rate_deg(1));
fprintf('q: %.6f\n', max_body_rate_deg(2));
fprintf('r: %.6f\n', max_body_rate_deg(3));

fprintf('\nMaximum spacecraft torque [N m]:\n');
fprintf('x: %.6f\n', max_spacecraft_torque(1));
fprintf('y: %.6f\n', max_spacecraft_torque(2));
fprintf('z: %.6f\n', max_spacecraft_torque(3));

fprintf('\nMaximum wheel speed [rpm]:\n');
fprintf('x: %.3f\n', max_wheel_speed_rpm(1));
fprintf('y: %.3f\n', max_wheel_speed_rpm(2));
fprintf('z: %.3f\n', max_wheel_speed_rpm(3));

fprintf('\nFinal wheel speed [rpm]:\n');
fprintf('x: %.6f\n', final_wheel_speed_rpm(1));
fprintf('y: %.6f\n', final_wheel_speed_rpm(2));
fprintf('z: %.6f\n', final_wheel_speed_rpm(3));

fprintf('\nMaximum wheel momentum [N m s]:\n');
fprintf('x: %.6f\n', max_wheel_momentum(1));
fprintf('y: %.6f\n', max_wheel_momentum(2));
fprintf('z: %.6f\n', max_wheel_momentum(3));

fprintf('\nWheel-speed limit activated [x y z]:\n');
disp(speed_limit_activated);

fprintf('Maximum quaternion norm error: %.3e\n', ...
    maximum_norm_error);

fprintf('Maximum total momentum magnitude: %.3e N m s\n', ...
    max(total_momentum_magnitude));

if isnan(settling_time)
    fprintf('The attitude did not settle within %.2f deg.\n', ...
        tolerance_deg);
else
    fprintf('Settling time within %.2f deg: %.3f s\n', ...
        tolerance_deg, settling_time);
end

%% Plot results
figure;

subplot(4,1,1);
plot(t, attitude_deg, 'LineWidth', 1.3);
grid on;
ylabel('Attitude [deg]');
title('Combined Three-Axis Control with Reaction Wheels');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');

subplot(4,1,2);
plot(t, omega_deg, 'LineWidth', 1.3);
grid on;
ylabel('Body rate [deg/s]');
legend('p', 'q', 'r', 'Location', 'best');

subplot(4,1,3);
plot(t, wheel_speed_rpm, 'LineWidth', 1.3);
grid on;
ylabel('Wheel speed [rpm]');
legend('\Omega_x', '\Omega_y', '\Omega_z', ...
    'Location', 'best');

subplot(4,1,4);
plot(t, spacecraft_torque, 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Torque [N m]');
legend('\tau_x', '\tau_y', '\tau_z', ...
    'Location', 'best');