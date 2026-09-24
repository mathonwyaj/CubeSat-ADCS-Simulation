%% Quaternion Attitude Control with Reaction Wheels

clear;
clc;
close all;

%% Load parameters
init_attitude_3axis;
init_reaction_wheels;
init_sensor_models;

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
%% Generate simulated sensor measurements

gyro_measured = zeros(number_of_samples, 3);
gyro_bias_history = zeros(number_of_samples, 3);

star_tracker_measured = ...
    NaN(number_of_samples, 4);

star_tracker_error_deg = ...
    NaN(number_of_samples, 1);

current_gyro_bias = gyro_bias_initial;

star_tracker_interval = round( ...
    star_tracker_sample_time / gyro_sample_time);

for k = 1:number_of_samples
    if k > 1
        current_gyro_bias = update_gyro_bias( ...
            current_gyro_bias, ...
            gyro_bias_random_walk, ...
            gyro_sample_time);
    end

    gyro_bias_history(k,:) = ...
        current_gyro_bias.';

    gyro_measured_k = gyro_measurement( ...
        omega(k,:).', ...
        current_gyro_bias, ...
        gyro_noise_std);

    gyro_measured(k,:) = ...
        gyro_measured_k.';

    if mod(k - 1, star_tracker_interval) == 0
        measured_q = star_tracker_measurement( ...
            quaternion_history(k,:).', ...
            star_tracker_noise_std);

        star_tracker_measured(k,:) = ...
            measured_q.';

        measurement_error_q = quaternion_error( ...
            measured_q, ...
            quaternion_history(k,:).');

        error_scalar = min(1, max(-1, ...
            abs(measurement_error_q(1))));

        star_tracker_error_deg(k) = ...
            rad2deg(2 * acos(error_scalar));
    end
end

%% Sensor-error statistics

gyro_error_deg_s = rad2deg( ...
    gyro_measured - omega);

gyro_error_mean_deg_s = ...
    mean(gyro_error_deg_s, 1);

gyro_error_std_deg_s = ...
    std(gyro_error_deg_s, 0, 1);

valid_star_tracker_samples = ...
    ~isnan(star_tracker_error_deg);

mean_star_tracker_error_deg = mean( ...
    star_tracker_error_deg(valid_star_tracker_samples));

rms_star_tracker_error_deg = sqrt(mean( ...
    star_tracker_error_deg(valid_star_tracker_samples).^2));

maximum_star_tracker_error_deg = max( ...
    star_tracker_error_deg(valid_star_tracker_samples));
%% Run the multiplicative extended Kalman filter

% Deliberately introduce an initial attitude-estimation error.
initial_estimation_error = ...
    deg2rad([2; -1; 1.5]);

initial_error_quaternion = ...
    rotation_vector_to_quaternion( ...
        initial_estimation_error);

initial_quaternion_estimate = ...
    quaternion_multiply( ...
        quaternion_history(1,:).', ...
        initial_error_quaternion);

% Begin without knowing the actual gyro bias.
initial_bias_estimate = [0; 0; 0];

% Initial estimator uncertainty
initial_attitude_sigma = deg2rad(5);
initial_bias_sigma = deg2rad(0.1);

initial_covariance = diag([
    initial_attitude_sigma^2 * ones(3,1);
    initial_bias_sigma^2 * ones(3,1)
]);

[quaternion_estimate_history, ...
 bias_estimate_history, ...
 covariance_diagonal_history, ...
 innovation_history] = run_mekf( ...
    gyro_measured, ...
    star_tracker_measured, ...
    gyro_sample_time, ...
    gyro_noise_std, ...
    gyro_bias_random_walk, ...
    star_tracker_noise_std, ...
    initial_quaternion_estimate, ...
    initial_bias_estimate, ...
    initial_covariance);

%% Calculate estimator errors

attitude_estimation_error_deg = ...
    zeros(number_of_samples, 1);
attitude_error_vector_deg = ...
    zeros(number_of_samples, 3);

for k = 1:number_of_samples
    estimate_error_quaternion = quaternion_error( ...
        quaternion_history(k,:).', ...
        quaternion_estimate_history(k,:).');
    attitude_error_vector_deg(k,:) = rad2deg( ...
    2 * estimate_error_quaternion(2:4)).';

    error_scalar = min(1, max(-1, ...
        abs(estimate_error_quaternion(1))));

    attitude_estimation_error_deg(k) = ...
        rad2deg(2 * acos(error_scalar));
end

bias_estimation_error_deg_s = rad2deg( ...
    bias_estimate_history - gyro_bias_history);

estimated_omega = ...
    gyro_measured - bias_estimate_history;

omega_estimation_error_deg_s = rad2deg( ...
    estimated_omega - omega);

%% Estimator performance measurements

final_attitude_estimation_error_deg = ...
    attitude_estimation_error_deg(end);

rms_attitude_estimation_error_deg = sqrt(mean( ...
    attitude_estimation_error_deg.^2));

maximum_attitude_estimation_error_deg = max( ...
    attitude_estimation_error_deg);

final_bias_estimate_deg_s = ...
    rad2deg(bias_estimate_history(end,:));

final_true_bias_deg_s = ...
    rad2deg(gyro_bias_history(end,:));

final_bias_error_deg_s = ...
    bias_estimation_error_deg_s(end,:);

rms_rate_estimation_error_deg_s = sqrt(mean( ...
    omega_estimation_error_deg_s.^2, 1));
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
%% Display MEKF results

fprintf('\nMEKF attitude-estimation error [deg]:\n');
fprintf('Final: %.6f\n', ...
    final_attitude_estimation_error_deg);
fprintf('RMS:   %.6f\n', ...
    rms_attitude_estimation_error_deg);
fprintf('Max:   %.6f\n', ...
    maximum_attitude_estimation_error_deg);

fprintf('\nFinal estimated gyro bias [deg/s]:\n');
fprintf('x: %.6f\n', final_bias_estimate_deg_s(1));
fprintf('y: %.6f\n', final_bias_estimate_deg_s(2));
fprintf('z: %.6f\n', final_bias_estimate_deg_s(3));

fprintf('\nFinal true gyro bias [deg/s]:\n');
fprintf('x: %.6f\n', final_true_bias_deg_s(1));
fprintf('y: %.6f\n', final_true_bias_deg_s(2));
fprintf('z: %.6f\n', final_true_bias_deg_s(3));

fprintf('\nFinal gyro-bias estimation error [deg/s]:\n');
fprintf('x: %.6f\n', final_bias_error_deg_s(1));
fprintf('y: %.6f\n', final_bias_error_deg_s(2));
fprintf('z: %.6f\n', final_bias_error_deg_s(3));

fprintf('\nRMS angular-rate estimation error [deg/s]:\n');
fprintf('x: %.6f\n', rms_rate_estimation_error_deg_s(1));
fprintf('y: %.6f\n', rms_rate_estimation_error_deg_s(2));
fprintf('z: %.6f\n', rms_rate_estimation_error_deg_s(3));

%% Display sensor statistics

fprintf('\nMean gyro measurement error [deg/s]:\n');
fprintf('x: %.6f\n', gyro_error_mean_deg_s(1));
fprintf('y: %.6f\n', gyro_error_mean_deg_s(2));
fprintf('z: %.6f\n', gyro_error_mean_deg_s(3));

fprintf('\nGyro measurement-error standard deviation [deg/s]:\n');
fprintf('x: %.6f\n', gyro_error_std_deg_s(1));
fprintf('y: %.6f\n', gyro_error_std_deg_s(2));
fprintf('z: %.6f\n', gyro_error_std_deg_s(3));

fprintf('\nStar-tracker attitude-error statistics [deg]:\n');
fprintf('Mean: %.6f\n', mean_star_tracker_error_deg);
fprintf('RMS:  %.6f\n', rms_star_tracker_error_deg);
fprintf('Max:  %.6f\n', maximum_star_tracker_error_deg);

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
%% Plot sensor measurements and errors

gyro_bias_history_deg_s = ...
    rad2deg(gyro_bias_history);

figure;

subplot(3,1,1);
plot(t, gyro_error_deg_s, 'LineWidth', 0.8);
grid on;
ylabel('Gyro error [deg/s]');
title('Simulated Sensor Measurements');
legend('x', 'y', 'z', 'Location', 'best');

subplot(3,1,2);
plot(t, gyro_bias_history_deg_s, 'LineWidth', 1.2);
grid on;
ylabel('Gyro bias [deg/s]');
legend('b_x', 'b_y', 'b_z', 'Location', 'best');

subplot(3,1,3);
plot( ...
    t(valid_star_tracker_samples), ...
    star_tracker_error_deg(valid_star_tracker_samples), ...
    '.', ...
    'MarkerSize', 8);

grid on;
xlabel('Time [s]');
ylabel('Attitude error [deg]');
legend('Star tracker', 'Location', 'best');
%% Plot MEKF estimation performance

attitude_three_sigma_deg = rad2deg( ...
    3 * sqrt(sum( ...
    covariance_diagonal_history(:,1:3), 2)));

bias_three_sigma_deg_s = rad2deg( ...
    3 * sqrt( ...
    covariance_diagonal_history(:,4:6)));

figure;

subplot(3,1,1);
plot(t, attitude_estimation_error_deg, ...
    'LineWidth', 1.2);
hold on;
plot(t, attitude_three_sigma_deg, '--', ...
    'LineWidth', 1.2);
grid on;
ylabel('Attitude error [deg]');
title('MEKF Estimation Performance');
legend('Estimation error', '3\sigma bound', ...
    'Location', 'best');

subplot(3,1,2);
plot(t, bias_estimation_error_deg_s, ...
    'LineWidth', 1.1);
grid on;
ylabel('Bias error [deg/s]');
legend('x', 'y', 'z', 'Location', 'best');

subplot(3,1,3);
plot(t, rad2deg(gyro_bias_history), ...
    'LineWidth', 1.2);
hold on;
plot(t, rad2deg(bias_estimate_history), '--', ...
    'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Gyro bias [deg/s]');
legend( ...
    'True x', 'True y', 'True z', ...
    'Estimated x', 'Estimated y', 'Estimated z', ...
    'Location', 'best');