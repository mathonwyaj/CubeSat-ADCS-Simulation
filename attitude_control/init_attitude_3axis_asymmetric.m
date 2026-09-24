%% CubeSat 3-Axis Attitude Control - Initial Parameters

clear;
clc;

%% Spacecraft properties
mass = 4.0;       % kg

width  = 0.10;    % m, body x dimension
depth  = 0.20;    % m, body y dimension
height = 0.30;    % m, body z dimension

%% Principal moments of inertia
% Rectangular cuboid about its centre of mass

Ixx = (1/12) * mass * (depth^2 + height^2);
Iyy = (1/12) * mass * (width^2 + height^2);
Izz = (1/12) * mass * (width^2 + depth^2);

I = diag([Ixx, Iyy, Izz]);

fprintf('Ixx = %.7f kg m^2\n', Ixx);
fprintf('Iyy = %.7f kg m^2\n', Iyy);
fprintf('Izz = %.7f kg m^2\n', Izz);

disp('Inertia matrix I [kg m^2]:');
disp(I);

%% Initial attitude [roll; pitch; yaw]
attitude0 = deg2rad([30; 0; 0]);

%% Initial body angular velocity [p; q; r]
omega0 = [0; 0; 0];

%% Commanded attitude [roll; pitch; yaw]
attitude_cmd = deg2rad([0; 0; 0]);

%% Per-axis PD gains
Kp = [0.0067; 0.0067; 0.0067];
Kd = [0.0107; 0.0107; 0.0107];

%% Per-axis actuator torque limits
tau_max = [0.002; 0.002; 0.002];

%% Simulation duration
t_final = 20;     % s