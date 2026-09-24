function [total_acceleration, two_body_acceleration, ...
    j2_acceleration] = orbital_acceleration_j2( ...
        position, mu, earth_radius, J2)
%ORBITAL_ACCELERATION_J2
% Calculate two-body and Earth J2 acceleration.
%
% Inputs:
%   position     - Earth-centred inertial position [x; y; z], m
%   mu           - Earth gravitational parameter, m^3/s^2
%   earth_radius - Earth reference radius, m
%   J2           - dimensionless second zonal harmonic
%
% Outputs:
%   total_acceleration    - two-body plus J2, m/s^2
%   two_body_acceleration - spherical-Earth gravity, m/s^2
%   j2_acceleration       - J2 perturbation, m/s^2

x = position(1);
y = position(2);
z = position(3);

radius = norm(position);
radius_squared = radius^2;
z_squared = z^2;

%% Two-body acceleration
two_body_acceleration = ...
    -mu * position / radius^3;

%% J2 acceleration
j2_factor = ...
    1.5 * J2 * mu * earth_radius^2 / radius^5;

common_xy_term = ...
    5 * z_squared / radius_squared - 1;

z_term = ...
    5 * z_squared / radius_squared - 3;

j2_acceleration = j2_factor * [
    x * common_xy_term;
    y * common_xy_term;
    z * z_term
];

total_acceleration = ...
    two_body_acceleration + j2_acceleration;

end