function [measured_quaternion, rotation_noise] = ...
    star_tracker_measurement( ...
        true_quaternion, attitude_noise_std)
%STAR_TRACKER_MEASUREMENT Simulate a noisy quaternion measurement.
%
% Inputs:
%   true_quaternion   - true attitude [qw; qx; qy; qz]
%   attitude_noise_std - standard deviation of each small-angle
%                        noise component in rad
%
% Outputs:
%   measured_quaternion - noisy unit quaternion
%   rotation_noise      - generated rotation-noise vector in rad

true_quaternion = ...
    true_quaternion / norm(true_quaternion);

%% Generate a small random rotation vector
rotation_noise = ...
    attitude_noise_std .* randn(3,1);

noise_angle = norm(rotation_noise);

%% Convert the noise rotation vector to a quaternion
if noise_angle > 0
    noise_axis = rotation_noise / noise_angle;

    noise_quaternion = [
        cos(noise_angle / 2);
        noise_axis * sin(noise_angle / 2)
    ];
else
    noise_quaternion = [1; 0; 0; 0];
end

%% Apply body-frame measurement error
measured_quaternion = quaternion_multiply( ...
    true_quaternion, noise_quaternion);

measured_quaternion = ...
    measured_quaternion / norm(measured_quaternion);

%% Keep the measured quaternion in the same hemisphere
if dot(measured_quaternion, true_quaternion) < 0
    measured_quaternion = -measured_quaternion;
end

end