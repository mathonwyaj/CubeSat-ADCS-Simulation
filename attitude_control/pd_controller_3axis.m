function [tau, tau_unsaturated, attitude_error] = ...
    pd_controller_3axis(attitude_cmd, attitude, omega, Kp, Kd, tau_max)
%PD_CONTROLLER_3AXIS Three-axis Euler-angle PD controller.
%
% Inputs:
%   attitude_cmd - commanded [roll; pitch; yaw] in rad
%   attitude     - current [roll; pitch; yaw] in rad
%   omega        - body rates [p; q; r] in rad/s
%   Kp           - proportional gains [Kp_x; Kp_y; Kp_z]
%   Kd           - derivative gains [Kd_x; Kd_y; Kd_z]
%   tau_max      - positive per-axis torque limits in N m
%
% Outputs:
%   tau              - saturated commanded torque in N m
%   tau_unsaturated  - torque before saturation in N m
%   attitude_error   - wrapped Euler-angle error in rad

raw_error = attitude_cmd - attitude;

% Wrap each error to the interval [-pi, pi].
attitude_error = atan2(sin(raw_error), cos(raw_error));

tau_unsaturated = ...
    Kp .* attitude_error - Kd .* omega;

% Apply saturation independently to each axis.
tau = max(min(tau_unsaturated, tau_max), -tau_max);

end