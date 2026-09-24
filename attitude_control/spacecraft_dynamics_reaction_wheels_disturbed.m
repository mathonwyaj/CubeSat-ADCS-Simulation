function state_dot = ...
    spacecraft_dynamics_reaction_wheels_disturbed( ...
        ~, state, requested_spacecraft_torque, ...
        external_torque, I, wheel_inertia, ...
        wheel_torque_max, wheel_speed_max)
%SPACECRAFT_DYNAMICS_REACTION_WHEELS_DISTURBED
% Quaternion spacecraft dynamics with reaction wheels and
% externally applied disturbance torque.
%
% State:
%   state(1:4)  = quaternion
%   state(5:7)  = body rates in rad/s
%   state(8:10) = wheel speeds in rad/s

quaternion = state(1:4);
omega = state(5:7);
wheel_speed = state(8:10);

quaternion = quaternion / norm(quaternion);

%% Reaction-wheel actuator
[spacecraft_control_torque, wheel_acceleration] = ...
    reaction_wheel_actuator( ...
        requested_spacecraft_torque, ...
        wheel_speed, ...
        wheel_inertia, ...
        wheel_torque_max, ...
        wheel_speed_max);

%% Quaternion kinematics
quaternion_dot = quaternion_kinematics( ...
    quaternion, omega);

%% Total internal angular momentum
wheel_momentum = ...
    wheel_inertia .* wheel_speed;

total_angular_momentum = ...
    I * omega + wheel_momentum;

%% Spacecraft rotational dynamics
omega_dot = I \ ( ...
    spacecraft_control_torque ...
    + external_torque ...
    - cross(omega, total_angular_momentum));

%% Complete state derivative
state_dot = [
    quaternion_dot;
    omega_dot;
    wheel_acceleration
];

end