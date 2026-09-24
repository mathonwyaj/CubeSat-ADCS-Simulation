%% Solar-Radiation-Pressure Disturbance-Rejection Test

clear;
clc;
close all;

%% Load asymmetric spacecraft and wheel parameters
init_attitude_3axis_asymmetric;
init_reaction_wheels;

%% Earth and orbit parameters
mu = 3.986004418e14;     % m^3/s^2
earth_radius = 6371e3;   % m
altitude = 500e3;        % m
orbital_radius = earth_radius + altitude;

orbital_rate = sqrt(mu / orbital_radius^3);
orbital_period = 2*pi / orbital_rate;

fprintf('Orbital period: %.3f min\n', ...
    orbital_period / 60);
orbital_speed = sqrt(mu / orbital_radius);

%% Simplified solar-radiation-pressure parameters
sun_direction_inertial = [1; 0; 0];

solar_pressure = 4.56e-6;       % N/m^2
reflectivity_coefficient = 1.5;
illuminated_area = 0.02;        % m^2
illumination_factor = 1;        % fully illuminated

solar_cp_offset_body = [
    0;
    0.02;
    0
];

%% Scale controller gains for asymmetric inertia
reference_inertia = 0.00666666666666667;
reference_Kp = 0.0067;
reference_Kd = 0.0107;

principal_inertia = diag(I);

Kp = reference_Kp ...
    * principal_inertia / reference_inertia;

Kd = reference_Kd ...
    * principal_inertia / reference_inertia;

%% Initial and commanded attitude
% The spacecraft begins at its command so the test isolates disturbance.
commanded_attitude = deg2rad([0; 0; 0]);

command_quaternion = ...
    euler_to_quaternion(commanded_attitude);

initial_quaternion = command_quaternion;

initial_state = [
    initial_quaternion;
    0;
    0;
    0;
    0;
    0;
    0
];

%% Simulation settings
sample_time = 0.1;       % s
simulation_duration = 600; % s

time = (0:sample_time:simulation_duration).';
number_of_samples = length(time);

%% Allocate histories
state_history = zeros(number_of_samples, 10);
srp_torque_history = zeros(number_of_samples, 3);
control_torque_history = zeros(number_of_samples, 3);
position_history = zeros(number_of_samples, 3);
pointing_error_deg = zeros(number_of_samples, 1);

state = initial_state;
state_history(1,:) = state.';

%% Fixed-step simulation
for k = 1:number_of_samples
    current_time = time(k);

    %% Circular-orbit position
    orbital_angle = orbital_rate * current_time;

    position_inertial = orbital_radius * [
        cos(orbital_angle);
        sin(orbital_angle);
        0
    ];

    position_history(k,:) = position_inertial.';

   %% Solar-radiation-pressure disturbance
[srp_torque_value, ~] = ...
    solar_radiation_pressure_torque( ...
        sun_direction_inertial, ...
        state(1:4), ...
        solar_pressure, ...
        reflectivity_coefficient, ...
        illuminated_area, ...
        solar_cp_offset_body, ...
        illumination_factor);

srp_torque_history(k,:) = ...
    srp_torque_value.';
    %% Perfect-state quaternion controller
    [~, requested_control_torque] = ...
        pd_controller_quaternion( ...
            command_quaternion, ...
            state(1:4), ...
            state(5:7), ...
            Kp, ...
            Kd, ...
            wheel_torque_max);

    [applied_control_torque, ~] = ...
        reaction_wheel_actuator( ...
            requested_control_torque, ...
            state(8:10), ...
            wheel_inertia, ...
            wheel_torque_max, ...
            wheel_speed_max);

    control_torque_history(k,:) = ...
        applied_control_torque.';

    %% Record pointing error
    error_quaternion = quaternion_error( ...
        command_quaternion, ...
        state(1:4));

    error_scalar = min(1, max(-1, ...
        abs(error_quaternion(1))));

    pointing_error_deg(k) = ...
        rad2deg(2 * acos(error_scalar));

    if k == number_of_samples
        break;
    end

    %% Propagate spacecraft, wheels, and disturbance
    state = rk4_reaction_wheel_disturbed_step( ...
        state, ...
        requested_control_torque, ...
        srp_torque_value, ...
        sample_time, ...
        I, ...
        wheel_inertia, ...
        wheel_torque_max, ...
        wheel_speed_max);

    state_history(k+1,:) = state.';
end

%% Derived quantities
wheel_speed_rpm = ...
    state_history(:,8:10) * 60 / (2*pi);

srp_torque_nNm = ...
    srp_torque_history * 1e9;

control_torque_nNm = ...
    control_torque_history * 1e9;

%% Performance results
maximum_pointing_error_deg = ...
    max(pointing_error_deg);

rms_pointing_error_deg = sqrt(mean( ...
    pointing_error_deg.^2));

maximum_srp_torque = ...
    max(abs(srp_torque_history), [], 1);

maximum_wheel_speed_rpm = ...
    max(abs(wheel_speed_rpm), [], 1);

final_wheel_speed_rpm = ...
    wheel_speed_rpm(end,:);

%% Display results
fprintf('\nSolar-radiation-pressure rejection results:\n');

fprintf('\nMaximum pointing error: %.6e deg\n', ...
    maximum_pointing_error_deg);

fprintf('RMS pointing error: %.6e deg\n', ...
    rms_pointing_error_deg);

fprintf('\nMaximum SRP torque [N m]:\n');
fprintf('x: %.6e\n', maximum_srp_torque(1));
fprintf('y: %.6e\n', maximum_srp_torque(2));
fprintf('z: %.6e\n', maximum_srp_torque(3));

fprintf('\nMaximum wheel speed [rpm]:\n');
fprintf('x: %.6f\n', maximum_wheel_speed_rpm(1));
fprintf('y: %.6f\n', maximum_wheel_speed_rpm(2));
fprintf('z: %.6f\n', maximum_wheel_speed_rpm(3));

fprintf('\nFinal wheel speed [rpm]:\n');
fprintf('x: %.6f\n', final_wheel_speed_rpm(1));
fprintf('y: %.6f\n', final_wheel_speed_rpm(2));
fprintf('z: %.6f\n', final_wheel_speed_rpm(3));

%% Plot results
figure;

subplot(3,1,1);
plot(time, pointing_error_deg, 'LineWidth', 1.2);
grid on;
ylabel('Pointing error [deg]');
title('Solar-Radiation-Pressure Disturbance Rejection');

subplot(3,1,2);
plot(time, srp_torque_nNm, 'LineWidth', 1.2);
grid on;
ylabel('SRP torque [nN m]');
legend('\tau_x', '\tau_y', '\tau_z', ...
    'Location', 'best');

subplot(3,1,3);
plot(time, wheel_speed_rpm, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Wheel speed [rpm]');
legend('\Omega_x', '\Omega_y', '\Omega_z', ...
    'Location', 'best');