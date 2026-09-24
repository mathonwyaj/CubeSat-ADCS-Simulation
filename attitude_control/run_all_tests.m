function test_summary = run_all_tests
%RUN_ALL_TESTS Automated CubeSat project validation.

clc;
close all;

fprintf('CubeSat project validation\n');
fprintf('==========================\n\n');

test_names = {
    'Parameter initialisation'
    'Nominal three-axis attitude control'
    'Estimated-state attitude control'
    'Three-axis Simulink model'
    'C implementation'
};

test_functions = {
    @test_initialisation
    @test_nominal_attitude_control
    @test_estimated_state_control
    @test_simulink_model
    @test_c_implementation
};

number_of_tests = numel(test_names);
test_passed = false(number_of_tests,1);
test_message = strings(number_of_tests,1);

for test_number = 1:number_of_tests
    fprintf('[%d/%d] %s ... ', ...
        test_number, number_of_tests, ...
        test_names{test_number});

    try
        test_functions{test_number}();

        test_passed(test_number) = true;
        test_message(test_number) = "Passed";

        fprintf('PASSED\n');

    catch test_error
        test_message(test_number) = ...
            string(test_error.message);

        fprintf('FAILED\n');
        fprintf('      %s\n', test_error.message);
    end
end

number_passed = sum(test_passed);
number_failed = number_of_tests - number_passed;

fprintf('\nValidation summary\n');
fprintf('==================\n');
fprintf('Tests passed: %d/%d\n', ...
    number_passed, number_of_tests);
fprintf('Tests failed: %d\n', number_failed);

test_summary = table( ...
    string(test_names), ...
    test_passed, ...
    test_message, ...
    'VariableNames', {
        'Test'
        'Passed'
        'Message'
    });

disp(test_summary);

if number_failed == 0
    fprintf('ALL PROJECT TESTS PASSED.\n');
else
    error('%d project test(s) failed.', ...
        number_failed);
end

end


function test_initialisation

init_attitude_3axis_asymmetric;
init_reaction_wheels;
init_sensor_models;

assert(isequal(size(I), [3 3]), ...
    'Spacecraft inertia must be 3-by-3.');

assert(all(diag(I) > 0), ...
    'Spacecraft inertia must be positive.');

assert(all(abs(wheel_inertia - 2e-5) < 1e-12), ...
    'Unexpected reaction-wheel inertia.');

assert(all(abs(wheel_torque_max - 0.002) < 1e-12), ...
    'Unexpected wheel-torque limit.');

expected_speed_limit = 6000 * 2*pi / 60;

assert(all(abs( ...
    wheel_speed_max - expected_speed_limit) < 1e-6), ...
    'Unexpected wheel-speed limit.');

assert(gyro_sample_time > 0, ...
    'Invalid gyroscope sample time.');

assert(star_tracker_sample_time > 0, ...
    'Invalid star-tracker sample time.');

end


function test_nominal_attitude_control

init_attitude_3axis_asymmetric;
init_reaction_wheels;

reference_inertia = 0.00666666666666667;
reference_Kp = 0.0067;
reference_Kd = 0.0107;

principal_inertia = diag(I);

Kp = reference_Kp ...
    * principal_inertia / reference_inertia;

Kd = reference_Kd ...
    * principal_inertia / reference_inertia;

results = simulate_monte_carlo_attitude_case( ...
    [30; -20; 15], ...
    I, ...
    Kp, ...
    Kd, ...
    wheel_inertia, ...
    wheel_torque_max, ...
    wheel_speed_max, ...
    0.02, ...
    20);

assert(results.final_error_deg < 0.01, ...
    'Nominal final pointing error is too large.');

assert(isfinite(results.settling_time), ...
    'Nominal case did not settle.');

assert(results.settling_time < 15, ...
    'Nominal settling time exceeds 15 seconds.');

assert(~any(results.wheel_speed_limit_activated), ...
    'A nominal reaction wheel reached its speed limit.');

assert(all(results.maximum_torque ...
    <= wheel_torque_max.' + 1e-12), ...
    'Applied torque exceeded the actuator limit.');

end


function test_estimated_state_control

init_attitude_3axis_asymmetric;
init_reaction_wheels;
init_sensor_models;

reference_inertia = 0.00666666666666667;
reference_Kp = 0.0067;
reference_Kd = 0.0107;

principal_inertia = diag(I);

Kp = reference_Kp ...
    * principal_inertia / reference_inertia;

Kd = reference_Kd ...
    * principal_inertia / reference_inertia;

rng(84);

results = simulate_estimated_state_case( ...
    [30; -20; 15], ...
    I, ...
    Kp, ...
    Kd, ...
    wheel_inertia, ...
    wheel_torque_max, ...
    wheel_speed_max, ...
    gyro_bias_initial, ...
    gyro_noise_std, ...
    gyro_bias_random_walk, ...
    star_tracker_noise_std, ...
    gyro_sample_time, ...
    star_tracker_sample_time, ...
    20, ...
    [3; -2; 1]);

assert(results.final_control_error_deg < 0.5, ...
    'Estimated-state control error exceeds 0.5 deg.');

assert(results.final_estimation_error_deg < 0.1, ...
    'Final estimation error exceeds 0.1 deg.');

assert(isfinite(results.settling_time), ...
    'Estimated-state case did not settle.');

assert(~any(results.speed_limit_activated), ...
    'A wheel-speed limit was activated.');

end


function test_simulink_model

assert(isfile('cubesat_attitude_3axis.slx'), ...
    'cubesat_attitude_3axis.slx was not found.');

evalin('base', 'init_simulink_3axis;');

simulation_output = ...
    sim('cubesat_attitude_3axis');

output_names = simulation_output.who;

required_outputs = {
    'state_3axis_out'
    'torque_3axis_out'
    'wheel_speed_3axis_out'
    'quaternion_norm_3axis_out'
};

for output_number = 1:numel(required_outputs)
    assert(any(strcmp( ...
        output_names, required_outputs{output_number})), ...
        ['Missing Simulink output: ', ...
        required_outputs{output_number}]);
end

quaternion_norm_output = ...
    simulation_output.quaternion_norm_3axis_out;

maximum_norm_error = max(abs( ...
    quaternion_norm_output.Data(:) - 1));

assert(maximum_norm_error < 1e-6, ...
    'Simulink quaternion norm error is too large.');

end


function test_c_implementation

current_folder = fileparts(mfilename('fullpath'));
project_folder = fileparts(current_folder);

c_folder = fullfile( ...
    project_folder, 'c_implementation');

build_folder = fullfile( ...
    c_folder, 'build');

assert(isfolder(c_folder), ...
    'The c_implementation folder was not found.');

assert(isfolder(build_folder), ...
    'The C build folder was not found.');

ctest_executable = ...
    'C:\Program Files\CMake\bin\ctest.exe';

assert(isfile(ctest_executable), ...
    ['ctest.exe was not found at: ', ...
    ctest_executable]);

command = sprintf( ...
    '"%s" --test-dir "%s" -C Release --output-on-failure', ...
    ctest_executable, ...
    build_folder);

[status, command_output] = system(command);

assert(status == 0, ...
    "CTest failed:" + newline + string(command_output));

assert(contains(command_output, ...
    '100% tests passed'), ...
    'CTest did not report 100 percent passing.');

end