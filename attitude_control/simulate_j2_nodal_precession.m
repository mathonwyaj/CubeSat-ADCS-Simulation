%% J2 Nodal-Precession Simulation

clear;
clc;
close all;

%% Earth parameters
mu = 3.986004418e14;       % m^3/s^2
earth_radius = 6371e3;     % m
J2 = 1.08262668e-3;

%% Initial orbit
altitude = 500e3;
orbital_radius = earth_radius + altitude;
initial_inclination = deg2rad(51.6);

orbital_speed = sqrt(mu / orbital_radius);

initial_position = [
    orbital_radius;
    0;
    0
];

initial_velocity = [
    0;
    orbital_speed * cos(initial_inclination);
    orbital_speed * sin(initial_inclination)
];

initial_state = [
    initial_position;
    initial_velocity
];

%% Simulation settings
sample_time = 10;                 % s
simulation_duration = 3*24*3600; % three days

time = (0:sample_time:simulation_duration).';
number_of_samples = length(time);

%% Allocate histories
state_j2_history = zeros(number_of_samples, 6);
state_two_body_history = zeros(number_of_samples, 6);

raan_j2 = zeros(number_of_samples, 1);
raan_two_body = zeros(number_of_samples, 1);

inclination_j2 = zeros(number_of_samples, 1);
inclination_two_body = zeros(number_of_samples, 1);

state_j2 = initial_state;
state_two_body = initial_state;

%% Propagation
for k = 1:number_of_samples
    state_j2_history(k,:) = state_j2.';
    state_two_body_history(k,:) = state_two_body.';

    [~, ~, inclination_j2(k), raan_j2(k)] = ...
        orbital_elements_from_state( ...
            state_j2(1:3), ...
            state_j2(4:6), ...
            mu);

    [~, ~, inclination_two_body(k), ...
        raan_two_body(k)] = ...
        orbital_elements_from_state( ...
            state_two_body(1:3), ...
            state_two_body(4:6), ...
            mu);

    if k == number_of_samples
        break;
    end

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
end

%% Unwrap angular histories
raan_j2_unwrapped = unwrap(raan_j2);
raan_two_body_unwrapped = unwrap(raan_two_body);

raan_j2_deg = rad2deg(raan_j2_unwrapped);
raan_two_body_deg = rad2deg(raan_two_body_unwrapped);

inclination_j2_deg = rad2deg(inclination_j2);
inclination_two_body_deg = ...
    rad2deg(inclination_two_body);

time_days = time / (24*3600);

%% Numerically measured RAAN drift rate
raan_fit = polyfit(time_days, raan_j2_deg, 1);
measured_raan_rate_deg_day = raan_fit(1);

%% Theoretical secular J2 RAAN rate
mean_motion = sqrt(mu / orbital_radius^3);

theoretical_raan_rate = ...
    -1.5 ...
    * J2 ...
    * mean_motion ...
    * (earth_radius / orbital_radius)^2 ...
    * cos(initial_inclination);

theoretical_raan_rate_deg_day = ...
    rad2deg(theoretical_raan_rate) ...
    * 24 * 3600;

%% Results
final_j2_raan_change = ...
    raan_j2_deg(end) - raan_j2_deg(1);

final_two_body_raan_change = ...
    raan_two_body_deg(end) ...
    - raan_two_body_deg(1);

fprintf('\nJ2 nodal-precession results:\n');

fprintf('Measured J2 RAAN rate: %.6f deg/day\n', ...
    measured_raan_rate_deg_day);

fprintf('Theoretical J2 RAAN rate: %.6f deg/day\n', ...
    theoretical_raan_rate_deg_day);

fprintf('Final J2 RAAN change: %.6f deg\n', ...
    final_j2_raan_change);

fprintf('Final two-body RAAN change: %.6e deg\n', ...
    final_two_body_raan_change);

fprintf('Mean J2 inclination: %.6f deg\n', ...
    mean(inclination_j2_deg));

%% Plot results
figure;

subplot(2,1,1);

plot(time_days, raan_two_body_deg, ...
    '--', 'LineWidth', 1.2);

hold on;

plot(time_days, raan_j2_deg, ...
    'LineWidth', 1.2);

grid on;
ylabel('RAAN [deg]');
title('J2 Nodal Precession');

legend( ...
    'Two body', ...
    'J2', ...
    'Location', 'best');

subplot(2,1,2);

plot(time_days, inclination_two_body_deg, ...
    '--', 'LineWidth', 1.2);

hold on;

plot(time_days, inclination_j2_deg, ...
    'LineWidth', 1.2);

grid on;
xlabel('Time [days]');
ylabel('Inclination [deg]');

legend( ...
    'Two body', ...
    'J2', ...
    'Location', 'best');