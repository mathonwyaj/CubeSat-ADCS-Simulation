function state_dot = spacecraft_dynamics_quaternion( ...
    ~, state, tau, I)
%SPACECRAFT_DYNAMICS_QUATERNION Seven-state spacecraft model.
%
% State:
%   state(1:4) = quaternion [qw; qx; qy; qz]
%   state(5:7) = body rates [p; q; r] in rad/s
%
% Inputs:
%   tau = applied body torque in N m
%   I   = spacecraft inertia matrix in kg m^2
%
% Output:
%   state_dot = [quaternion derivative; angular acceleration]

quaternion = state(1:4);
omega = state(5:7);

% Use a normalized quaternion in the kinematic calculation.
quaternion = quaternion / norm(quaternion);

quaternion_dot = quaternion_kinematics( ...
    quaternion, omega);

omega_dot = rigid_body_dynamics( ...
    omega, tau, I);

state_dot = [quaternion_dot; omega_dot];

end