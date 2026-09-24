%% Orbit Propagation with J2 Perturbation

clear;
clc;
close all;

%% Earth and orbit parameters
mu = 3.986004418e14;       % m^3/s^2
earth_radius = 6371e3;     % m
altitude = 500e3;          % m
orbital_radius = earth_radius + altitude;
J2 = 1.08262668e-3;

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

sample_time = 1;                    % s
number_of_orbits = 3;
simulation_duration = ...
    number_of_orbits * orbital_period;

time = (0:sample_time:simulation_duration).';
number_of_samples = length(time);

fprintf('Orbital period: %.3f min\n', ...
    orbital_period / 60);

fprintf('Simulation duration: %.3f hours\n', ...
    simulation_duration / 3600);

%% Allocate histories
state_j2_history = zeros(number_of_samples, 6);
state_two_body_history = zeros(number_of_samples, 6);

state_j2 = initial_state;
state_two_body = initial_state;

state_j2_history(1,:) = state_j2.';
state_two_body_history(1,:) = state_two_body.';

%% Propagate both orbit models
for k = 1:number_of_samples-1
    current_step = time(k+1) - time(k);

    state_j2 = rk4_orbit_j2_step( ...
        state_j2, ...
        current_step, ...
        mu, ...
        earth_radius, ...
        J2);

    state_two_body = rk4_orbit_j2_step( ...
        state_two_body, ...
        current_step, ...
        mu, ...
        earth_radius, ...
        0);

    state_j2_history(k+1,:) = state_j2.';
    state_two_body_history(k+1,:) = state_two_body.';
end

%% Derived quantities
radius_j2 = vecnorm( ...
    state_j2_history(:,1:3), 2, 2);

radius_two_body = vecnorm( ...
    state_two_body_history(:,1:3), 2, 2);

altitude_j2 = radius_j2 - earth_radius;
altitude_two_body = radius_two_body - earth_radius;

position_difference = vecnorm( ...
    state_j2_history(:,1:3) ...
    - state_two_body_history(:,1:3), ...
    2, 2);

%% Performance results
minimum_altitude_j2 = min(altitude_j2);
maximum_altitude_j2 = max(altitude_j2);

final_position_difference = ...
    position_difference(end);

maximum_position_difference = ...
    max(position_difference);

fprintf('\nJ2 orbit results:\n');

fprintf('Minimum altitude: %.3f km\n', ...
    minimum_altitude_j2 / 1e3);

fprintf('Maximum altitude: %.3f km\n', ...
    maximum_altitude_j2 / 1e3);

fprintf('Final J2/two-body position difference: %.3f km\n', ...
    final_position_difference / 1e3);

fprintf('Maximum J2/two-body position difference: %.3f km\n', ...
    maximum_position_difference / 1e3);

%% Plot three-dimensional trajectories
figure;

plot3( ...
    state_two_body_history(:,1) / 1e3, ...
    state_two_body_history(:,2) / 1e3, ...
    state_two_body_history(:,3) / 1e3, ...
    '--', ...
    'LineWidth', 1.2);

hold on;

plot3( ...
    state_j2_history(:,1) / 1e3, ...
    state_j2_history(:,2) / 1e3, ...
    state_j2_history(:,3) / 1e3, ...
    'LineWidth', 1.2);

grid on;
axis equal;

xlabel('x [km]');
ylabel('y [km]');
zlabel('z [km]');

title('Two-Body and J2 Orbit Propagation');

legend( ...
    'Two body', ...
    'J2', ...
    'Location', 'best');

%% Plot altitude and position difference
figure;

subplot(2,1,1);

plot(time / 60, altitude_two_body / 1e3, ...
    '--', 'LineWidth', 1.2);

hold on;

plot(time / 60, altitude_j2 / 1e3, ...
    'LineWidth', 1.2);

grid on;
ylabel('Altitude [km]');
title('Effect of J2 on Orbit Propagation');

legend( ...
    'Two body', ...
    'J2', ...
    'Location', 'best');

subplot(2,1,2);

plot(time / 60, position_difference / 1e3, ...
    'LineWidth', 1.2);

grid on;
xlabel('Time [min]');
ylabel('Position difference [km]');