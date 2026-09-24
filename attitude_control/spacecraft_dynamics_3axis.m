function state_dot = spacecraft_dynamics_3axis(~, state, tau, I)
%SPACECRAFT_DYNAMICS_3AXIS Six-state rigid-body attitude model.
%
% State:
%   state(1:3) = Euler angles [roll; pitch; yaw] in rad
%   state(4:6) = body rates [p; q; r] in rad/s
%
% Inputs:
%   tau = applied body torque [tau_x; tau_y; tau_z] in N m
%   I   = spacecraft inertia matrix in kg m^2
%
% Output:
%   state_dot = [Euler-angle rates; body angular accelerations]

euler_angles = state(1:3);
omega = state(4:6);

euler_dot = euler_kinematics(euler_angles, omega);
omega_dot = rigid_body_dynamics(omega, tau, I);

state_dot = [euler_dot; omega_dot];

end