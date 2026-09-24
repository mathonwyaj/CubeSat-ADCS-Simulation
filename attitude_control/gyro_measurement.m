function [measured_omega, noise] = ...
    gyro_measurement(true_omega, gyro_bias, gyro_noise_std)
%GYRO_MEASUREMENT Simulate a three-axis gyroscope measurement.
%
% Inputs:
%   true_omega     - true body rates [p; q; r] in rad/s
%   gyro_bias      - gyro bias in rad/s
%   gyro_noise_std - white-noise standard deviation in rad/s
%
% Outputs:
%   measured_omega - simulated gyro measurement in rad/s
%   noise          - generated white-noise sample in rad/s

noise = gyro_noise_std .* randn(size(true_omega));

measured_omega = ...
    true_omega + gyro_bias + noise;

end