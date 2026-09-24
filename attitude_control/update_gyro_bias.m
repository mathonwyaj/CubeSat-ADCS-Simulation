function next_bias = update_gyro_bias( ...
    current_bias, bias_random_walk, sample_time)
%UPDATE_GYRO_BIAS Propagate gyro bias using a random walk.
%
% Inputs:
%   current_bias    - current three-axis gyro bias in rad/s
%   bias_random_walk - random-walk strength in rad/s/sqrt(s)
%   sample_time     - update interval in seconds
%
% Output:
%   next_bias       - updated three-axis bias in rad/s

bias_increment = ...
    bias_random_walk * sqrt(sample_time) .* randn(3,1);

next_bias = current_bias + bias_increment;

end