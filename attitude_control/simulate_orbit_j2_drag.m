%% Long-Duration Orbit Propagation with J2 and Atmospheric Drag

clear;
clc;
close all;

%% Earth and orbit parameters
mu = 3.986004418e14;       % m^3/s^2
earth_radius = 6371e3;     % m
altitude = 500e3;          % m
orbital_radius = earth_radius + altitude;

J2 = 1.08262668e-3;
earth_rotation_rate = 7.2921159e-5; % rad/s

%% Spacecraft and simplified atmospheric parameters
spacecraft_mass = 4;             % kg
atmospheric_density = 1.0e-12;   % kg/m^3
drag_coefficient = 2.2;
reference_area = 0.02;           % m^2

%% Initial circular orbit
initial_position = [
    orbital_radius;
    0;
    0
];

initial_velocity = [
    0;
    sqrt(mu / orbital_radius);
    0
];

initial_state = [
    initial_position;
    initial_velocity
];

%% Simulation settings
orbital_period = 2*pi ...
    * sqrt(orbital_radius^3 / mu);

sample_time = 10;                 % s
simulation_duration = 7*24*3600;  % seven days

time = (0:sample_time:simulation_duration).';
number_of_samples = length(time);

fprintf('Orbital period: %.3f min\n', ...
    orbital_period / 60);

fprintf('Simulation duration: %.3f days\n', ...
    simulation_duration / (24*3600));

fprintf('Number of integration steps: %d\n', ...
    number_of_samples - 1);

%% Allocate histories
state_j2_history = ...
    zeros(number_of_samples, 6);

state_j2_drag_history = ...
    zeros(number_of_samples, 6);

state_j2 = initial_state;
state_j2_drag = initial_state;

state_j2_history(1,:) = state_j2.';
state_j2_drag_history(1,:) = state_j2_drag.';

%% Propagate both models
for k = 1:number_of_samples-1
    current_step = time(k+1) - time(k);

    % J2 without drag
    state_j2 = rk4_orbit_j2_step( ...
        state_j2, ...
        current_step, ...
        mu, ...
        earth_radius, ...
        J2);

    % J2 with drag
    state_j2_drag = ...
        rk4_orbit_j2_drag_step( ...
            state_j2_drag, ...
            current_step, ...
            mu, ...
            earth_radius, ...
            J2, ...
            spacecraft_mass, ...
            atmospheric_density, ...
            drag_coefficient, ...
            reference_area, ...
            earth_rotation_rate);

    state_j2_history(k+1,:) = state_j2.';
    state_j2_drag_history(k+1,:) = ...
        state_j2_drag.';
end

%% Radius and altitude
radius_j2 = vecnorm( ...
    state_j2_history(:,1:3), 2, 2);

radius_j2_drag = vecnorm( ...
    state_j2_drag_history(:,1:3), 2, 2);

altitude_j2 = radius_j2 - earth_radius;
altitude_j2_drag = radius_j2_drag - earth_radius;

%% Osculating semi-major axis
speed_squared_j2 = sum( ...
    state_j2_history(:,4:6).^2, 2);

speed_squared_j2_drag = sum( ...
    state_j2_drag_history(:,4:6).^2, 2);

specific_energy_j2 = ...
    0.5 * speed_squared_j2 ...
    - mu ./ radius_j2;

specific_energy_j2_drag = ...
    0.5 * speed_squared_j2_drag ...
    - mu ./ radius_j2_drag;

semi_major_axis_j2 = ...
    -mu ./ (2 * specific_energy_j2);

semi_major_axis_j2_drag = ...
    -mu ./ (2 * specific_energy_j2_drag);

%% Differences caused by drag
semi_major_axis_difference = ...
    semi_major_axis_j2_drag ...
    - semi_major_axis_j2;

position_difference = vecnorm( ...
    state_j2_drag_history(:,1:3) ...
    - state_j2_history(:,1:3), ...
    2, 2);

final_semi_major_axis_change = ...
    semi_major_axis_j2_drag(end) ...
    - semi_major_axis_j2_drag(1);

final_drag_relative_axis_difference = ...
    semi_major_axis_difference(end);

final_position_difference = ...
    position_difference(end);

%% Display results
fprintf('\nJ2 and drag orbit results:\n');

fprintf('\nJ2-only final altitude: %.3f km\n', ...
    altitude_j2(end) / 1e3);

fprintf('J2-plus-drag final altitude: %.3f km\n', ...
    altitude_j2_drag(end) / 1e3);

fprintf('\nJ2-plus-drag semi-major-axis change: %.3f m\n', ...
    final_semi_major_axis_change);

fprintf('Final drag-relative semi-major-axis difference: %.3f m\n', ...
    final_drag_relative_axis_difference);

fprintf('Final position difference: %.3f km\n', ...
    final_position_difference / 1e3);

%% Plot results
time_days = time / (24*3600);

figure;

subplot(3,1,1);

plot(time_days, altitude_j2 / 1e3, ...
    '--', 'LineWidth', 1.0);

hold on;

plot(time_days, altitude_j2_drag / 1e3, ...
    'LineWidth', 1.0);

grid on;
ylabel('Altitude [km]');
title('Long-Duration J2 and Drag Orbit Comparison');

legend( ...
    'J2 only', ...
    'J2 and drag', ...
    'Location', 'best');

subplot(3,1,2);

plot(time_days, ...
    semi_major_axis_difference, ...
    'LineWidth', 1.2);

grid on;
ylabel('\Deltaa [m]');
title('Semi-Major-Axis Difference Caused by Drag');

subplot(3,1,3);

plot(time_days, ...
    position_difference / 1e3, ...
    'LineWidth', 1.2);

grid on;
xlabel('Time [days]');
ylabel('Position difference [km]');