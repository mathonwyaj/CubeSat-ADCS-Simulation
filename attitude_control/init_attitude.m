%% CubeSat Attitude Control - Initial Parameters

clear;
clc;

% CubeSat properties
mass = 4;          % kg
width = 0.10;      % m
height = 0.10;     % m

% Moment of inertia about the rotation axis
Izz = (1/12) * mass * (width^2 + height^2);

% Initial spacecraft attitude
theta0 = deg2rad(30);     % rad - spacecraft starts 30 degrees off target
omega0 = 0;               % rad/s - initially not rotating

% Desired attitude
theta_cmd = 0;            % rad - target orientation

% PD controller gains
Kp = 0.0067;
Kd = 0.0107;

% Maximum actuator torque
tau_max = 0.002;           % N*m

% Simulation duration
t_final = 20;              % seconds

% Display calculated moment of inertia
disp(Izz)