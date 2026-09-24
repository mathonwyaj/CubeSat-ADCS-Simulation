%% CubeSat Sensor-Model Parameters

%% Reproducible random sequence
random_seed = 42;
rng(random_seed);

%% Gyroscope parameters
gyro_sample_rate = 100;            % Hz
gyro_sample_time = 1 / gyro_sample_rate;

% Constant initial bias assumption
gyro_bias_initial_deg_s = [
     0.05;
    -0.03;
     0.02
];                                 % deg/s

gyro_bias_initial = ...
    deg2rad(gyro_bias_initial_deg_s); % rad/s

% Per-sample white-noise standard deviation
gyro_noise_std_deg_s = 0.02;        % deg/s

gyro_noise_std = ...
    deg2rad(gyro_noise_std_deg_s);  % rad/s

% Bias random-walk parameter for a later dynamic-bias model
gyro_bias_random_walk_deg_s_sqrt_s = 0.0005;

gyro_bias_random_walk = deg2rad( ...
    gyro_bias_random_walk_deg_s_sqrt_s); % rad/s/sqrt(s)

%% Star-tracker parameters for the later attitude measurement model
star_tracker_sample_rate = 5;       % Hz
star_tracker_sample_time = ...
    1 / star_tracker_sample_rate;

star_tracker_noise_std_deg = 0.01;  % deg

star_tracker_noise_std = ...
    deg2rad(star_tracker_noise_std_deg); % rad

%% Display parameters
disp('Gyroscope initial bias [deg/s]:');
disp(gyro_bias_initial_deg_s);

fprintf('Gyroscope noise standard deviation: %.4f deg/s\n', ...
    gyro_noise_std_deg_s);

fprintf('Gyroscope sample rate: %.1f Hz\n', ...
    gyro_sample_rate);

fprintf('Star-tracker noise standard deviation: %.4f deg\n', ...
    star_tracker_noise_std_deg);

fprintf('Star-tracker sample rate: %.1f Hz\n', ...
    star_tracker_sample_rate);