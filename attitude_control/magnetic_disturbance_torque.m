function [torque_body, magnetic_field_body] = ...
    magnetic_disturbance_torque( ...
        residual_dipole_body, ...
        magnetic_field_inertial, ...
        quaternion)
%MAGNETIC_DISTURBANCE_TORQUE
% Calculate disturbance torque from residual spacecraft dipole.
%
% Inputs:
%   residual_dipole_body   - residual dipole [A m^2], body axes
%   magnetic_field_inertial - magnetic field [T], inertial axes
%   quaternion             - body-to-inertial quaternion
%
% Outputs:
%   torque_body            - magnetic torque [N m], body axes
%   magnetic_field_body    - magnetic field [T], body axes

body_to_inertial = ...
    quaternion_to_rotation_matrix(quaternion);

magnetic_field_body = ...
    body_to_inertial.' * magnetic_field_inertial;

torque_body = cross( ...
    residual_dipole_body, ...
    magnetic_field_body);

end