function [spacecraft_torque, wheel_acceleration, ...
    wheel_motor_torque, speed_limited] = ...
    reaction_wheel_actuator( ...
        requested_spacecraft_torque, ...
        wheel_speed, ...
        wheel_inertia, ...
        wheel_torque_max, ...
        wheel_speed_max)
%REACTION_WHEEL_ACTUATOR Three orthogonal reaction-wheel model.
%
% Inputs:
%   requested_spacecraft_torque - controller request in N m
%   wheel_speed                 - wheel speeds in rad/s
%   wheel_inertia               - rotor inertias in kg m^2
%   wheel_torque_max            - motor torque limits in N m
%   wheel_speed_max             - wheel speed limits in rad/s
%
% Outputs:
%   spacecraft_torque - torque applied to spacecraft in N m
%   wheel_acceleration - wheel acceleration in rad/s^2
%   wheel_motor_torque - torque applied to each wheel in N m
%   speed_limited      - logical flag for each wheel

%% Equal-and-opposite wheel torque request
requested_wheel_torque = -requested_spacecraft_torque;

%% Apply motor torque limits
wheel_motor_torque = max( ...
    min(requested_wheel_torque, wheel_torque_max), ...
    -wheel_torque_max);

%% Prevent acceleration farther beyond the speed limit
speed_limited = false(3,1);

for axis = 1:3
    at_positive_limit = ...
        wheel_speed(axis) >= wheel_speed_max(axis);

    at_negative_limit = ...
        wheel_speed(axis) <= -wheel_speed_max(axis);

    accelerating_positive = wheel_motor_torque(axis) > 0;
    accelerating_negative = wheel_motor_torque(axis) < 0;

    if (at_positive_limit && accelerating_positive) || ...
       (at_negative_limit && accelerating_negative)

        wheel_motor_torque(axis) = 0;
        speed_limited(axis) = true;
    end
end

%% Wheel acceleration
wheel_acceleration = ...
    wheel_motor_torque ./ wheel_inertia;

%% Equal-and-opposite spacecraft torque
spacecraft_torque = -wheel_motor_torque;

end