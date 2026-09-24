function torque_body = gravity_gradient_torque( ...
    position_inertial, quaternion, I, mu)
%GRAVITY_GRADIENT_TORQUE Calculate gravity-gradient torque.
%
% Inputs:
%   position_inertial - Earth-centred position vector in m
%   quaternion        - body-to-inertial attitude quaternion
%   I                 - spacecraft inertia matrix in kg m^2
%   mu                - Earth gravitational parameter in m^3/s^2
%
% Output:
%   torque_body       - gravity-gradient torque in body axes, N m

orbital_radius = norm(position_inertial);

radial_unit_inertial = ...
    position_inertial / orbital_radius;

body_to_inertial = ...
    quaternion_to_rotation_matrix(quaternion);

% Convert the radial direction from inertial to body coordinates.
radial_unit_body = ...
    body_to_inertial.' * radial_unit_inertial;

torque_body = ...
    3 * mu / orbital_radius^3 ...
    * cross( ...
        radial_unit_body, ...
        I * radial_unit_body);

end