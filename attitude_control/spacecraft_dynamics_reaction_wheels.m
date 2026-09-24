function state_dot = spacecraft_dynamics_reaction_wheels( ...
    ~, state, requested_spacecraft_torque, I, ...
    wheel_inertia, wheel_torque_max, wheel_speed_max)
%SPACECRAFT_DYNAMICS_REACTION_WHEELS
% Quaternion spacecraft dynamics with three orthogonal reaction wheels.
%
% State:
%   state(1:4)  = quaternion [qw; qx; qy; qz]
%   state(5:7)  = body rates [p; q; r] in rad/s
%   state(8:10) = wheel speeds in rad/s

quaternion = state(1:4);
omega = state(5:7);
wheel_speed = state(8:10);

quaternion = quaternion / norm(quaternion);

%% Reaction-wheel actuator
[spacecraft_torque, wheel_acceleration] = ...
    reaction_wheel_actuator( ...
        requested_spacecraft_torque, ...
        wheel_speed, ...
        wheel_inertia, ...
        wheel_torque_max, ...
        wheel_speed_max);

%% Quaternion kinematics
quaternion_dot = quaternion_kinematics( ...
    quaternion, omega);

%% Wheel angular momentum in body coordinates
wheel_momentum = wheel_inertia .* wheel_speed;

%% Spacecraft angular acceleration
total_angular_momentum = ...
    I * omega + wheel_momentum;

omega_dot = I \ ( ...
    spacecraft_torque ...
    - cross(omega, total_angular_momentum));

%% Complete state derivative
state_dot = [
    quaternion_dot;
    omega_dot;
    wheel_acceleration
];

end