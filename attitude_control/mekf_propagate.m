function [quaternion_predicted, bias_predicted, ...
    covariance_predicted] = mekf_propagate( ...
        quaternion_estimate, ...
        bias_estimate, ...
        covariance, ...
        gyro_measurement_value, ...
        sample_time, ...
        gyro_noise_std, ...
        bias_random_walk)
%MEKF_PROPAGATE Propagate quaternion, gyro bias, and covariance.
%
% Error state:
%   error_state = [attitude_error; bias_error]
%
% Each component is three-dimensional.

quaternion_estimate = ...
    quaternion_estimate / norm(quaternion_estimate);

%% Remove estimated gyro bias
corrected_omega = ...
    gyro_measurement_value - bias_estimate;

%% Propagate nominal quaternion
rotation_increment = ...
    corrected_omega * sample_time;

delta_quaternion = ...
    rotation_vector_to_quaternion(rotation_increment);

quaternion_predicted = quaternion_multiply( ...
    quaternion_estimate, delta_quaternion);

quaternion_predicted = ...
    quaternion_predicted / norm(quaternion_predicted);

%% Bias follows a random-walk model
% Its expected value remains unchanged during propagation.
bias_predicted = bias_estimate;

%% Linearized continuous-time error dynamics
omega_skew = [
     0,                  -corrected_omega(3),  corrected_omega(2);
     corrected_omega(3),  0,                 -corrected_omega(1);
    -corrected_omega(2),  corrected_omega(1),  0
];

continuous_state_matrix = [
    -omega_skew, -eye(3);
     zeros(3),    zeros(3)
];

%% First-order discrete transition matrix
transition_matrix = ...
    eye(6) + continuous_state_matrix * sample_time;

%% Discrete process-noise covariance
attitude_process_noise = ...
    (gyro_noise_std * sample_time)^2 * eye(3);

bias_process_noise = ...
    (bias_random_walk^2 * sample_time) * eye(3);

process_noise_covariance = [
    attitude_process_noise, zeros(3);
    zeros(3), bias_process_noise
];

%% Covariance propagation
covariance_predicted = ...
    transition_matrix * covariance ...
    * transition_matrix.' ...
    + process_noise_covariance;

% Enforce numerical symmetry.
covariance_predicted = 0.5 * ( ...
    covariance_predicted + covariance_predicted.');

end