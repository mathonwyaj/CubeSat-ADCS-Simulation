%% Three-Axis Closed-Loop Attitude-Control Simulation

clear;
clc;
close all;

%% Load spacecraft and controller parameters
init_attitude_3axis;

%% Initial six-state vector
% [roll; pitch; yaw; p; q; r]
initial_state = [attitude0; omega0];

%% Output times
time = (0:0.01:t_final).';

%% Solver settings
solver_options = odeset( ...
    'RelTol', 1e-10, ...
    'AbsTol', 1e-12);

%% Run closed-loop simulation
[t, state] = ode45( ...
    @(time, state) closed_loop_dynamics_3axis( ...
        time, state, attitude_cmd, Kp, Kd, tau_max, I), ...
    time, ...
    initial_state, ...
    solver_options);

%% Extract states
attitude = state(:, 1:3);
omega = state(:, 4:6);

%% Recalculate controller outputs for recording
number_of_samples = length(t);

torque = zeros(number_of_samples, 3);
attitude_error = zeros(number_of_samples, 3);

for k = 1:number_of_samples
    [torque_k, ~, error_k] = pd_controller_3axis( ...
        attitude_cmd, ...
        attitude(k,:).', ...
        omega(k,:).', ...
        Kp, ...
        Kd, ...
        tau_max);

    torque(k,:) = torque_k.';
    attitude_error(k,:) = error_k.';
end

%% Convert angular quantities for display
attitude_deg = rad2deg(attitude);
omega_deg = rad2deg(omega);
attitude_error_deg = rad2deg(attitude_error);

%% Performance measurements
final_error_deg = attitude_error_deg(end,:);
max_rate_deg = max(abs(omega_deg), [], 1);
max_torque = max(abs(torque), [], 1);

tolerance_deg = 0.5;
settling_time = NaN;

for k = 1:number_of_samples
    remaining_error = abs(attitude_error_deg(k:end,:));

    if all(remaining_error(:) <= tolerance_deg)
        settling_time = t(k);
        break;
    end
end

%% Display results
fprintf('\nFinal attitude error [deg]:\n');
fprintf('Roll:  %.6f\n', final_error_deg(1));
fprintf('Pitch: %.6f\n', final_error_deg(2));
fprintf('Yaw:   %.6f\n', final_error_deg(3));

fprintf('\nMaximum body rate [deg/s]:\n');
fprintf('p: %.6f\n', max_rate_deg(1));
fprintf('q: %.6f\n', max_rate_deg(2));
fprintf('r: %.6f\n', max_rate_deg(3));

fprintf('\nMaximum control torque [N m]:\n');
fprintf('x: %.6f\n', max_torque(1));
fprintf('y: %.6f\n', max_torque(2));
fprintf('z: %.6f\n', max_torque(3));

if isnan(settling_time)
    fprintf('\nThe attitude did not settle within +/- %.2f deg.\n', ...
        tolerance_deg);
else
    fprintf('\nSettling time within +/- %.2f deg: %.3f s\n', ...
        tolerance_deg, settling_time);
end

%% Plot results
figure;

subplot(3,1,1);
plot(t, attitude_deg, 'LineWidth', 1.4);
grid on;
ylabel('Attitude [deg]');
title('Three-Axis Closed-Loop Attitude Response');
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