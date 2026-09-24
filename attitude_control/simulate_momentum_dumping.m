%% Reaction-Wheel Momentum Dumping with Magnetorquers

clear;
clc;
close all;

%% Load spacecraft and reaction-wheel parameters
init_attitude_3axis_asymmetric;
init_reaction_wheels;

%% Earth and orbit parameters
mu = 3.986004418e14;
earth_radius = 6371e3;
altitude = 500e3;
orbital_radius = earth_radius + altitude;

orbital_rate = sqrt(mu / orbital_radius^3);
orbital_period = 2*pi / orbital_rate;

orbit_inclination = deg2rad(51.6);

equatorial_surface_field = 3.12e-5;
dipole_axis_inertial = [0; 0; 1];

fprintf('Orbital period: %.3f min\n', ...
    orbital_period / 60);

%% Momentum-dumping parameters
dumping_gain = 1e-3;       % 1/s
maximum_dipole = 0.2;      % A m^2 per axis

%% Inertia-scaled attitude-controller gains
reference_inertia = 0.00666666666666667;
reference_Kp = 0.0067;
reference_Kd = 0.0107;

principal_inertia = diag(I);

Kp = reference_Kp ...
    * principal_inertia / reference_inertia;

Kd = reference_Kd ...
    * principal_inertia / reference_inertia;

%% Commanded attitude
command_quaternion = [1; 0; 0; 0];

%% Initial reaction-wheel speeds
initial_wheel_speed_rpm = [
     3000;
    -2000;
     1000
];

initial_wheel_speed = ...
    initial_wheel_speed_rpm * 2*pi / 60;

%% Initial plant state
initial_state = [
    command_quaternion;
    0;
    0;
    0;
    initial_wheel_speed
];

%% Simulation settings
sample_time = 0.2;
simulation_duration = 2 * orbital_period;

time = (0:sample_time:simulation_duration).';
number_of_samples = length(time);

%% Allocate histories
state_history = zeros(number_of_samples, 10);
magnetic_field_history = zeros(number_of_samples, 3);
magnetic_torque_history = zeros(number_of_samples, 3);
dipole_history = zeros(number_of_samples, 3);
wheel_momentum_history = zeros(number_of_samples, 3);
pointing_error_deg = zeros(number_of_samples, 1);

state = initial_state;
state_history(1,:) = state.';

%% Fixed-step simulation
for k = 1:number_of_samples
    current_time = time(k);

    %% Analytic inclined circular-orbit position
    orbital_angle = orbital_rate * current_time;

    position_inertial = orbital_radius * [
        cos(orbital_angle);
        cos(orbit_inclination) * sin(orbital_angle);
        sin(orbit_inclination) * sin(orbital_angle)
    ];

    %% Earth magnetic field in inertial axes
    magnetic_field_inertial = earth_dipole_field( ...
        position_inertial, ...
        earth_radius, ...
        equatorial_surface_field, ...
        dipole_axis_inertial);

    %% Convert magnetic field into body axes
    rotation_body_to_inertial = ...
        quaternion_to_rotation_matrix(state(1:4));

    magnetic_field_body = ...
        rotation_body_to_inertial.' ...
        * magnetic_field_inertial;

    magnetic_field_history(k,:) = ...
        magnetic_field_body.';

    %% Magnetorquer momentum-dumping command
    [commanded_dipole, magnetic_torque, ~, ...
        wheel_momentum] = ...
        magnetorquer_momentum_dump( ...
            state(8:10), ...
            wheel_inertia, ...
            magnetic_field_body, ...
            dumping_gain, ...
            maximum_dipole);

    dipole_history(k,:) = commanded_dipole.';
    magnetic_torque_history(k,:) = ...
        magnetic_torque.';
    wheel_momentum_history(k,:) = ...
        wheel_momentum.';

    %% Attitude-control torque
    [~, attitude_control_torque] = ...
        pd_controller_quaternion( ...
            command_quaternion, ...
            state(1:4), ...
            state(5:7), ...
            Kp, ...
            Kd, ...
            wheel_torque_max);

    % Reaction wheels counteract the magnetorquer torque while
    % simultaneously changing their stored momentum.
    requested_wheel_control_torque = ...
        attitude_control_torque ...
        - magnetic_torque;

    %% Pointing error
    error_quaternion = quaternion_error( ...
        command_quaternion, ...
        state(1:4));

    error_scalar = min(1, max(-1, ...
        abs(error_quaternion(1))));

    pointing_error_deg(k) = ...
        rad2deg(2 * acos(error_scalar));

    if k == number_of_samples
        break;
    end

    %% Propagate spacecraft and reaction wheels
    state = rk4_reaction_wheel_disturbed_step( ...
        state, ...
        requested_wheel_control_torque, ...
        magnetic_torque, ...
        sample_time, ...
        I, ...
        wheel_inertia, ...
        wheel_torque_max, ...
        wheel_speed_max);

    state_history(k+1,:) = state.';
end

%% Derived quantities
wheel_speed_rpm = ...
    state_history(:,8:10) * 60 / (2*pi);

wheel_momentum_magnitude = vecnorm( ...
    wheel_momentum_history, 2, 2);

magnetic_field_microtesla = ...
    magnetic_field_history * 1e6;

%% Performance results
final_wheel_speed_rpm = wheel_speed_rpm(end,:).';

initial_momentum_magnitude = ...
    wheel_momentum_magnitude(1);

final_momentum_magnitude = ...
    wheel_momentum_magnitude(end);

momentum_reduction_percent = ...
    100 * (1 - final_momentum_magnitude ...
    / initial_momentum_magnitude);

maximum_pointing_error = ...
    max(pointing_error_deg);

maximum_commanded_dipole = ...
    max(abs(dipole_history), [], 1);

wheel_speed_limit_activated = any( ...
    abs(state_history(:,8:10)) ...
    >= wheel_speed_max.' * (1 - 1e-9), 1);

%% Display results
fprintf('\nMomentum-dumping results:\n');

fprintf('\nInitial wheel speed [rpm]:\n');
fprintf('x: %.3f\n', initial_wheel_speed_rpm(1));
fprintf('y: %.3f\n', initial_wheel_speed_rpm(2));
fprintf('z: %.3f\n', initial_wheel_speed_rpm(3));

fprintf('\nFinal wheel speed [rpm]:\n');
fprintf('x: %.3f\n', final_wheel_speed_rpm(1));
fprintf('y: %.3f\n', final_wheel_speed_rpm(2));
fprintf('z: %.3f\n', final_wheel_speed_rpm(3));

fprintf('\nWheel-momentum magnitude:\n');
fprintf('Initial: %.6e N m s\n', ...
    initial_momentum_magnitude);
fprintf('Final:   %.6e N m s\n', ...
    final_momentum_magnitude);
fprintf('Reduction: %.3f percent\n', ...
    momentum_reduction_percent);

fprintf('\nMaximum pointing error: %.6e deg\n', ...
    maximum_pointing_error);

fprintf('\nMaximum commanded dipole [A m^2]:\n');
fprintf('x: %.6f\n', maximum_commanded_dipole(1));
fprintf('y: %.6f\n', maximum_commanded_dipole(2));
fprintf('z: %.6f\n', maximum_commanded_dipole(3));

fprintf('\nWheel-speed limit activated [x y z]:\n');
fprintf('   %d   %d   %d\n', ...
    wheel_speed_limit_activated);

%% Plot results
time_orbits = time / orbital_period;

figure;

subplot(4,1,1);

plot(time_orbits, wheel_speed_rpm, ...
    'LineWidth', 1.2);

grid on;
ylabel('Wheel speed [rpm]');
title('Reaction-Wheel Momentum Dumping');

legend('\Omega_x', '\Omega_y', '\Omega_z', ...
    'Location', 'best');

subplot(4,1,2);

plot(time_orbits, wheel_momentum_magnitude, ...
    'LineWidth', 1.2);

grid on;
ylabel('|h_w| [N m s]');

subplot(4,1,3);

plot(time_orbits, pointing_error_deg, ...
    'LineWidth', 1.2);

grid on;
ylabel('Pointing error [deg]');

subplot(4,1,4);

plot(time_orbits, dipole_history, ...
    'LineWidth', 1.2);

grid on;
xlabel('Time [orbits]');
ylabel('Dipole [A m^2]');

legend('m_x', 'm_y', 'm_z', ...
    'Location', 'best');