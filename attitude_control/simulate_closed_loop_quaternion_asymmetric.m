%% Quaternion Closed-Loop Attitude-Control Simulation

clear;
clc;
close all;

%% Load symmetric spacecraft parameters
init_attitude_3axis_asymmetric;

%% Scale gains using the original cube as the reference
reference_inertia = 0.00666666666666667;
reference_Kp = 0.0067;
reference_Kd = 0.0107;

principal_inertia = diag(I);

Kp = reference_Kp * principal_inertia / reference_inertia;
Kd = reference_Kd * principal_inertia / reference_inertia;

t_final = 60;

%% Convert initial and commanded attitudes to quaternions
combined_attitude0 = deg2rad([30; -20; 15]);

initial_quaternion = ...
    euler_to_quaternion(combined_attitude0);

command_quaternion = ...
    euler_to_quaternion(attitude_cmd);

%% Initial state [quaternion; body rates]
initial_state = [initial_quaternion; omega0];

%% Output times and solver settings
time = (0:0.01:t_final).';

solver_options = odeset( ...
    'RelTol', 1e-10, ...
    'AbsTol', 1e-12);

%% Run simulation
[t, state] = ode45( ...
    @(time, state) closed_loop_dynamics_quaternion( ...
        time, ...
        state, ...
        command_quaternion, ...
        Kp, ...
        Kd, ...
        tau_max, ...
        I), ...
    time, ...
    initial_state, ...
    solver_options);

%% Extract states
quaternion_history_raw = state(:,1:4);
omega = state(:,5:7);

%% Check and normalize quaternion history
quaternion_norm = sqrt(sum(quaternion_history_raw.^2, 2));

maximum_norm_error = max(abs(quaternion_norm - 1));

quaternion_history = ...
    quaternion_history_raw ./ quaternion_norm;

%% Convert quaternions to Euler angles for plotting
number_of_samples = length(t);

attitude = zeros(number_of_samples, 3);
torque = zeros(number_of_samples, 3);
attitude_error_angle_deg = zeros(number_of_samples, 1);

for k = 1:number_of_samples
    current_quaternion = quaternion_history(k,:).';

    attitude(k,:) = rad2deg( ...
        quaternion_to_euler(current_quaternion)).';

    [torque_k, ~, error_quaternion] = ...
        pd_controller_quaternion( ...
            command_quaternion, ...
            current_quaternion, ...
            omega(k,:).', ...
            Kp, ...
            Kd, ...
            tau_max);

    torque(k,:) = torque_k.';

    error_scalar = min(1, max(-1, abs(error_quaternion(1))));

    attitude_error_angle_deg(k) = ...
        rad2deg(2 * acos(error_scalar));
end

omega_deg = rad2deg(omega);

%% Performance measurements
final_attitude_deg = attitude(end,:);
final_error_angle_deg = attitude_error_angle_deg(end);

max_rate_deg = max(abs(omega_deg), [], 1);
max_torque = max(abs(torque), [], 1);

tolerance_deg = 0.5;
settling_time = NaN;

for k = 1:number_of_samples
    if all(attitude_error_angle_deg(k:end) <= tolerance_deg)
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
fprintf('p: %.6f\n', max_rate_deg(1));
fprintf('q: %.6f\n', max_rate_deg(2));
fprintf('r: %.6f\n', max_rate_deg(3));

fprintf('\nMaximum control torque [N m]:\n');
fprintf('x: %.6f\n', max_torque(1));
fprintf('y: %.6f\n', max_torque(2));
fprintf('z: %.6f\n', max_torque(3));

fprintf('\nMaximum quaternion norm error: %.3e\n', ...
    maximum_norm_error);

if isnan(settling_time)
    fprintf('\nThe attitude did not settle within %.2f deg.\n', ...
        tolerance_deg);
else
    fprintf('\nSettling time within %.2f deg: %.3f s\n', ...
        tolerance_deg, settling_time);
end

%% Plot results
figure;

subplot(3,1,1);
plot(t, attitude, 'LineWidth', 1.4);
grid on;
ylabel('Attitude [deg]');
title('Asymmetric Quaternion Attitude Response');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');

subplot(3,1,2);
plot(t, omega_deg, 'LineWidth', 1.4);
grid on;
ylabel('Body rate [deg/s]');
legend('p', 'q', 'r', 'Location', 'best');

subplot(3,1,3);
plot(t, torque, 'LineWidth', 1.4);
grid on;
xlabel('Time [s]');
ylabel('Torque [N m]');
legend('\tau_x', '\tau_y', '\tau_z', 'Location', 'best');