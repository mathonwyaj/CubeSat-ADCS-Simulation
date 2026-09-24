function [tau, tau_unsaturated, error_quaternion] = ...
    pd_controller_quaternion( ...
        command_quaternion, current_quaternion, omega, ...
        Kp, Kd, tau_max)
%PD_CONTROLLER_QUATERNION Quaternion-based three-axis PD controller.
%
% Inputs:
%   command_quaternion - desired attitude [qw; qx; qy; qz]
%   current_quaternion - current attitude [qw; qx; qy; qz]
%   omega              - body rates [p; q; r] in rad/s
%   Kp                 - proportional gains
%   Kd                 - derivative gains
%   tau_max            - positive per-axis torque limits
%
% Outputs:
%   tau                - saturated torque command in N m
%   tau_unsaturated    - torque before saturation in N m
%   error_quaternion   - quaternion attitude error

error_quaternion = quaternion_error( ...
    command_quaternion, current_quaternion);

error_vector = error_quaternion(2:4);

tau_unsaturated = ...
    2 * Kp .* error_vector - Kd .* omega;

tau = max(min(tau_unsaturated, tau_max), -tau_max);

end