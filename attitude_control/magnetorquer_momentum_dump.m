function [commanded_dipole, magnetic_torque, ...
    desired_dump_torque, wheel_momentum] = ...
    magnetorquer_momentum_dump( ...
        wheel_speed, ...
        wheel_inertia, ...
        magnetic_field_body, ...
        dumping_gain, ...
        maximum_dipole)
%MAGNETORQUER_MOMENTUM_DUMP Command magnetic momentum unloading.
%
% Inputs:
% wheel_speed        - reaction-wheel speeds [rad/s]
% wheel_inertia      - wheel inertias [kg m^2]
% magnetic_field_body - magnetic field in body axes [T]
% dumping_gain       - momentum-dumping gain [1/s]
% maximum_dipole     - maximum dipole per axis [A m^2]
%
% Outputs:
% commanded_dipole   - commanded magnetic dipole [A m^2]
% magnetic_torque    - achieved external torque [N m]
% desired_dump_torque - desired unloading torque [N m]
% wheel_momentum     - stored wheel momentum [N m s]

wheel_speed = wheel_speed(:);
wheel_inertia = wheel_inertia(:);
magnetic_field_body = magnetic_field_body(:);

if isscalar(maximum_dipole)
    maximum_dipole = ...
        maximum_dipole * ones(3,1);
else
    maximum_dipole = maximum_dipole(:);
end

%% Stored reaction-wheel momentum
wheel_momentum = ...
    wheel_inertia .* wheel_speed;

%% Desired external torque
desired_dump_torque = ...
    -dumping_gain * wheel_momentum;

magnetic_field_squared = ...
    dot(magnetic_field_body, ...
        magnetic_field_body);

%% No magnetic authority if the field is effectively zero
if magnetic_field_squared < 1e-20
    commanded_dipole = zeros(3,1);
    magnetic_torque = zeros(3,1);
    return;
end

%% Minimum-magnitude dipole command
commanded_dipole = ...
    cross(magnetic_field_body, ...
        desired_dump_torque) ...
    / magnetic_field_squared;

%% Apply per-axis dipole limits
commanded_dipole = min( ...
    max(commanded_dipole, ...
        -maximum_dipole), ...
    maximum_dipole);

%% Achieved magnetorquer torque
magnetic_torque = ...
    cross(commanded_dipole, ...
        magnetic_field_body);

end