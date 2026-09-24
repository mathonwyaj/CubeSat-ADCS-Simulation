%% Three-Axis Reaction-Wheel Parameters

%% Wheel arrangement
% Three mutually perpendicular wheels aligned with the body x, y and z axes.

number_of_wheels = 3;

%% Wheel rotor inertia
% Assumed value for the initial model.
wheel_inertia = [
    2.0e-5;
    2.0e-5;
    2.0e-5
];                              % kg m^2

%% Maximum wheel motor torque
wheel_torque_max = [
    0.002;
    0.002;
    0.002
];                              % N m

%% Maximum wheel speed
wheel_speed_max_rpm = [
    6000;
    6000;
    6000
];                              % rpm

wheel_speed_max = ...
    wheel_speed_max_rpm * 2*pi / 60;  % rad/s

%% Maximum stored angular momentum
wheel_momentum_max = ...
    wheel_inertia .* wheel_speed_max; % N m s

%% Display parameters
disp('Reaction-wheel inertia [kg m^2]:');
disp(wheel_inertia);

disp('Maximum wheel torque [N m]:');
disp(wheel_torque_max);

disp('Maximum wheel speed [rad/s]:');
disp(wheel_speed_max);

disp('Maximum wheel momentum [N m s]:');
disp(wheel_momentum_max);