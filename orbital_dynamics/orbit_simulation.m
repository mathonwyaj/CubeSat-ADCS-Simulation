%% CubeSat Orbital Dynamics Simulation

clear;
clc;
close all;

% Earth parameters
mu = 3.986004418e14;    % Earth's gravitational parameter (m^3/s^2)
R_E = 6371e3;           % Mean Earth radius (m)

% CubeSat orbit
altitude = 500e3;       % Orbital altitude (m)
r0 = R_E + altitude;    % Distance from Earth's centre (m)

% Circular orbital velocity
v0 = sqrt(mu / r0);

% Circular orbital period
T = 2*pi*sqrt(r0^3 / mu);

% Display results
fprintf('Orbital altitude: %.0f km\n', altitude/1000);
fprintf('Circular orbital velocity: %.3f km/s\n', v0/1000);
fprintf('Orbital period: %.2f minutes\n', T/60);

%% Initial spacecraft state

% Initial position vector
r_initial = [r0; 0; 0];

% Initial velocity vector
v_initial = [0; v0; 0];

% Combined state vector
state0 = [r_initial; v_initial];

% Simulation time - one complete orbit
tspan = [0 T];

%% Numerical orbit propagation

% Solve the equations of motion using ode45
% Increase numerical integration accuracy
options = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);

[t, state] = ode45(@(t, state) two_body_dynamics(t, state, mu), ...
                   tspan, state0, options);
%% Plot propagated orbit

% Extract spacecraft position
x = state(:,1);
y = state(:,2);
z = state(:,3);

% Plot orbit
figure;
plot3(x/1000, y/1000, z/1000, 'LineWidth', 1.5);
hold on;

% Plot Earth
[Xe, Ye, Ze] = sphere(50);
surf((R_E/1000)*Xe, ...
     (R_E/1000)*Ye, ...
     (R_E/1000)*Ze);

axis equal;
grid on;
xlabel('X (km)');
ylabel('Y (km)');
zlabel('Z (km)');
title('CubeSat 500 km Low Earth Orbit');
%% Validate numerical orbit

% Calculate distance from Earth's centre at each time step
r_mag = sqrt(x.^2 + y.^2 + z.^2);

% Convert radius to altitude above Earth's surface
altitude_numerical = (r_mag - R_E) / 1000;

% Calculate altitude variation
min_altitude = min(altitude_numerical);
max_altitude = max(altitude_numerical);

fprintf('\n--- Numerical Orbit Validation ---\n');
fprintf('Minimum altitude: %.3f km\n', min_altitude);
fprintf('Maximum altitude: %.3f km\n', max_altitude);
fprintf('Altitude variation: %.3f km\n', ...
        max_altitude - min_altitude);
% Save orbit figure
results_folder = fullfile('..', 'results');

if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

exportgraphics(gcf, fullfile(results_folder, 'orbit_500km.png'), ...
    'Resolution', 300);

%% Two-body orbital dynamics function

function dstate = two_body_dynamics(~, state, mu)

% Extract position and velocity
r = state(1:3);
v = state(4:6);

% Distance from Earth's centre
r_mag = norm(r);

% Two-body gravitational acceleration
a = -mu * r / r_mag^3;

% Return state derivative
dstate = [v; a];

end