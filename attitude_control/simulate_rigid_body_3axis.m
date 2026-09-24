%% Three-Axis Rigid-Body Dynamics: Torque-Free Symmetric Test

clear;
clc;
close all;

%% Load spacecraft parameters
init_attitude_3axis;

%% Initial body angular velocity
omega_initial = deg2rad([5; 10; 15]);   % rad/s

%% No applied torque
tau = [0; 0; 0];                        % N m

%% Simulation settings
t_span = [0, t_final];

solver_options = odeset( ...
    'RelTol', 1e-10, ...
    'AbsTol', 1e-12);

%% Propagate angular velocity
[t, omega] = ode45( ...
    @(t, omega) rigid_body_dynamics(omega, tau, I), ...
    t_span, ...
    omega_initial, ...
    solver_options);

%% Convert angular velocity to degrees per second for plotting
omega_deg = rad2deg(omega);

%% Measure numerical change from the initial angular velocity
omega_change = omega - omega_initial.';

max_rate_change = max(abs(omega_change), [], 1);

fprintf('\nMaximum angular-velocity change:\n');
fprintf('p: %.3e rad/s\n', max_rate_change(1));
fprintf('q: %.3e rad/s\n', max_rate_change(2));
fprintf('r: %.3e rad/s\n', max_rate_change(3));

%% Plot results
figure;

plot(t, omega_deg(:,1), 'LineWidth', 1.5);
hold on;
plot(t, omega_deg(:,2), 'LineWidth', 1.5);
plot(t, omega_deg(:,3), 'LineWidth', 1.5);

grid on;
xlabel('Time [s]');
ylabel('Angular velocity [deg/s]');
title('Torque-Free Rotation of a Symmetric Cube');
legend('p', 'q', 'r', 'Location', 'best');