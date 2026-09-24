%% Monte Carlo Attitude-Control Robustness Test

clear;
clc;
close all;

%% Load nominal parameters
init_attitude_3axis_asymmetric;
init_reaction_wheels;

%% Nominal controller gains
reference_inertia = 0.00666666666666667;
reference_Kp = 0.0067;
reference_Kd = 0.0107;

principal_inertia = diag(I);

nominal_Kp = reference_Kp ...
    * principal_inertia / reference_inertia;

nominal_Kd = reference_Kd ...
    * principal_inertia / reference_inertia;

%% Monte Carlo settings
number_of_cases = 100;
sample_time = 0.02;
simulation_duration = 20;

rng(42);

%% Allocate results
final_error_deg = zeros(number_of_cases, 1);
settling_time = NaN(number_of_cases, 1);

maximum_wheel_speed_rpm = ...
    zeros(number_of_cases, 3);

maximum_torque = zeros(number_of_cases, 3);

speed_limit_activated = ...
    false(number_of_cases, 3);

initial_attitude_history = ...
    zeros(number_of_cases, 3);

inertia_scale_history = ...
    zeros(number_of_cases, 3);

wheel_inertia_scale_history = ...
    zeros(number_of_cases, 3);

torque_scale_history = ...
    zeros(number_of_cases, 3);

%% Run randomized cases
for case_number = 1:number_of_cases
    %% Random initial attitude: nominal plus or minus 5 degrees
    initial_attitude_case = ...
        [30; -20; 15] ...
        + 5 * (2*rand(3,1) - 1);

    %% Spacecraft inertia uncertainty: plus or minus 10 percent
    inertia_scale = ...
        0.9 + 0.2 * rand(3,1);

    inertia_case = diag( ...
        diag(I) .* inertia_scale);

    %% Wheel-inertia uncertainty: plus or minus 10 percent
    wheel_inertia_scale = ...
        0.9 + 0.2 * rand(3,1);

    wheel_inertia_case = ...
        wheel_inertia .* wheel_inertia_scale;

    %% Available wheel-torque uncertainty: plus or minus 10 percent
    torque_scale = ...
        0.9 + 0.2 * rand(3,1);

    wheel_torque_case = ...
        wheel_torque_max .* torque_scale;

    %% Run one case
    case_results = ...
        simulate_monte_carlo_attitude_case( ...
            initial_attitude_case, ...
            inertia_case, ...
            nominal_Kp, ...
            nominal_Kd, ...
            wheel_inertia_case, ...
            wheel_torque_case, ...
            wheel_speed_max, ...
            sample_time, ...
            simulation_duration);

    %% Store inputs
    initial_attitude_history(case_number,:) = ...
        initial_attitude_case.';

    inertia_scale_history(case_number,:) = ...
        inertia_scale.';

    wheel_inertia_scale_history(case_number,:) = ...
        wheel_inertia_scale.';

    torque_scale_history(case_number,:) = ...
        torque_scale.';

    %% Store results
    final_error_deg(case_number) = ...
        case_results.final_error_deg;

    settling_time(case_number) = ...
        case_results.settling_time;

    maximum_wheel_speed_rpm(case_number,:) = ...
        case_results.maximum_wheel_speed_rpm;

    maximum_torque(case_number,:) = ...
        case_results.maximum_torque;

    speed_limit_activated(case_number,:) = ...
        case_results.wheel_speed_limit_activated;
end

%% Determine successful cases
settled_successfully = isfinite(settling_time);

final_accuracy_success = ...
    final_error_deg <= 0.5;

wheel_speed_success = ...
    ~any(speed_limit_activated, 2);

successful_case = ...
    settled_successfully ...
    & final_accuracy_success ...
    & wheel_speed_success;

success_rate_percent = ...
    100 * mean(successful_case);

%% Summary statistics
valid_settling_times = ...
    settling_time(isfinite(settling_time));

worst_final_error_deg = ...
    max(final_error_deg);

if isempty(valid_settling_times)
    mean_settling_time = NaN;
    worst_settling_time = NaN;
else
    mean_settling_time = ...
        mean(valid_settling_times);

    worst_settling_time = ...
        max(valid_settling_times);
end

overall_maximum_wheel_speed = ...
    max(maximum_wheel_speed_rpm, [], 1);

number_of_speed_limit_cases = ...
    sum(any(speed_limit_activated, 2));

%% Identify worst case
[~, worst_case_index] = ...
    max(final_error_deg);

%% Display results
fprintf('\nMonte Carlo attitude-control results:\n');

fprintf('Number of cases: %d\n', ...
    number_of_cases);

fprintf('Successful cases: %d\n', ...
    sum(successful_case));

fprintf('Success rate: %.1f percent\n', ...
    success_rate_percent);

fprintf('\nFinal pointing error [deg]:\n');
fprintf('Mean: %.6e\n', mean(final_error_deg));
fprintf('Worst: %.6e\n', worst_final_error_deg);

fprintf('\nSettling time [s]:\n');
fprintf('Mean: %.3f\n', mean_settling_time);
fprintf('Worst: %.3f\n', worst_settling_time);

fprintf('\nMaximum wheel speed across all cases [rpm]:\n');
fprintf('x: %.3f\n', overall_maximum_wheel_speed(1));
fprintf('y: %.3f\n', overall_maximum_wheel_speed(2));
fprintf('z: %.3f\n', overall_maximum_wheel_speed(3));

fprintf('\nCases activating a wheel-speed limit: %d\n', ...
    number_of_speed_limit_cases);

fprintf('\nWorst-final-error case number: %d\n', ...
    worst_case_index);

fprintf('Worst-case initial attitude [deg]:\n');
fprintf('Roll:  %.3f\n', ...
    initial_attitude_history(worst_case_index,1));
fprintf('Pitch: %.3f\n', ...
    initial_attitude_history(worst_case_index,2));
fprintf('Yaw:   %.3f\n', ...
    initial_attitude_history(worst_case_index,3));

fprintf('Worst-case inertia scales [x y z]:\n');
fprintf('%.4f  %.4f  %.4f\n', ...
    inertia_scale_history(worst_case_index,:));

%% Plot results
figure;

subplot(3,1,1);

histogram(final_error_deg, 15);

grid on;
xlabel('Final pointing error [deg]');
ylabel('Number of cases');
title('Monte Carlo Attitude-Control Robustness');

subplot(3,1,2);

histogram(valid_settling_times, 15);

grid on;
xlabel('Settling time [s]');
ylabel('Number of cases');

subplot(3,1,3);

plot(1:number_of_cases, ...
    maximum_wheel_speed_rpm(:,1), ...
    '.', 'MarkerSize', 10);

hold on;

plot(1:number_of_cases, ...
    maximum_wheel_speed_rpm(:,2), ...
    '.', 'MarkerSize', 10);

plot(1:number_of_cases, ...
    maximum_wheel_speed_rpm(:,3), ...
    '.', 'MarkerSize', 10);

yline(6000, '--r', 'Speed limit');

grid on;
xlabel('Case number');
ylabel('Maximum wheel speed [rpm]');

legend('x', 'y', 'z', 'Limit', ...
    'Location', 'best');