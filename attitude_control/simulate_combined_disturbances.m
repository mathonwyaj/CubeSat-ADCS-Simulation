%% Combined Environmental Disturbance-Rejection Test

clear;
clc;
close all;

%% Load asymmetric spacecraft and wheel parameters
init_attitude_3axis_asymmetric;
init_reaction_wheels;

%% Orbit parameters
environment.mu = 3.986004418e14;
environment.earth_radius = 6371e3;

altitude = 500e3;
orbital_radius = environment.earth_radius + altitude;

orbital_rate = sqrt( ...
    environment.mu / orbital_radius^3);

orbital_speed = sqrt( ...
    environment.mu / orbital_radius);

orbital_period = 2*pi / orbital_rate;

fprintf('Orbital period: %.3f min\n', ...
    orbital_period / 60);

%% Aerodynamic parameters
environment.atmospheric_density = 1.0e-12;
environment.drag_coefficient = 2.2;
environment.aerodynamic_area = 0.02;

environment.aerodynamic_cp_offset_body = [
    0.02;
    0;
    0
];

%% Solar-radiation-pressure parameters
environment.sun_direction_inertial = [1; 0; 0];
environment.solar_pressure = 4.56e-6;
environment.reflectivity_coefficient = 1.5;
environment.illuminated_area = 0.02;
environment.illumination_factor = 1;

environment.solar_cp_offset_body = [
    0;
    0.02;
    0
];

%% Magnetic parameters
environment.equatorial_surface_field = 3.12e-5;
environment.dipole_axis_inertial = [0; 0; 1];

environment.residual_dipole_body = [
    0.001;
    0;
    0
];

%% Inertia-scaled controller gains
reference_inertia = 0.00666666666666667;
reference_Kp = 0.0067;
reference_Kd = 0.0107;

principal_inertia = diag(I);

Kp = reference_Kp ...
    * principal_inertia / reference_inertia;

Kd = reference_Kd ...
    * principal_inertia / reference_inertia;

%% Initial and commanded state
command_quaternion = [1; 0; 0; 0];

state = [
    command_quaternion;
    0;
    0;
    0;
    0;
    0;
    0
];

%% Simulation settings
sample_time = 0.1;
simulation_duration = 600;

time = (0:sample_time:simulation_duration).';
number_of_samples = length(time);

%% Allocate histories
state_history = zeros(number_of_samples, 10);

total_torque_history = ...
    zeros(number_of_samples, 3);

gravity_torque_history = ...
    zeros(number_of_samples, 3);

aerodynamic_torque_history = ...
    zeros(number_of_samples, 3);

srp_torque_history = ...
    zeros(number_of_samples, 3);

magnetic_torque_history = ...
    zeros(number_of_samples, 3);

pointing_error_deg = ...
    zeros(number_of_samples, 1);

state_history(1,:) = state.';

%% Fixed-step simulation
for k = 1:number_of_samples
    orbital_angle = orbital_rate * time(k);

    position_inertial = orbital_radius * [
        cos(orbital_angle);
        sin(orbital_angle);
        0
    ];

    velocity_inertial = orbital_speed * [
        -sin(orbital_angle);
         cos(orbital_angle);
         0
    ];

    %% Combined environmental disturbance
    [total_environmental_torque, components] = ...
        combined_environmental_torque( ...
            position_inertial, ...
            velocity_inertial, ...
            state(1:4), ...
            I, ...
            environment);

    total_torque_history(k,:) = ...
        total_environmental_torque.';

    gravity_torque_history(k,:) = ...
        components.gravity_gradient.';

    aerodynamic_torque_history(k,:) = ...
        components.aerodynamic.';

    srp_torque_history(k,:) = ...
        components.srp.';

    magnetic_torque_history(k,:) = ...
        components.magnetic.';

    %% Perfect-state controller
    [~, requested_control_torque] = ...
        pd_controller_quaternion( ...
            command_quaternion, ...
            state(1:4), ...
            state(5:7), ...
            Kp, ...
            Kd, ...
            wheel_torque_max);

    %% Pointing error
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

    %% Propagate disturbed spacecraft
    state = rk4_reaction_wheel_disturbed_step( ...
        state, ...
        requested_control_torque, ...
        total_environmental_torque, ...
        sample_time, ...
        I, ...
        wheel_inertia, ...
        wheel_torque_max, ...
        wheel_speed_max);

    state_history(k+1,:) = state.';
end

%% Derived results
wheel_speed_rpm = ...
    state_history(:,8:10) * 60 / (2*pi);

total_torque_nNm = ...
    total_torque_history * 1e9;

gravity_magnitude_nNm = ...
    vecnorm(gravity_torque_history, 2, 2) * 1e9;

aerodynamic_magnitude_nNm = ...
    vecnorm(aerodynamic_torque_history, 2, 2) * 1e9;

srp_magnitude_nNm = ...
    vecnorm(srp_torque_history, 2, 2) * 1e9;

magnetic_magnitude_nNm = ...
    vecnorm(magnetic_torque_history, 2, 2) * 1e9;

%% Performance measurements
maximum_pointing_error_deg = ...
    max(pointing_error_deg);

rms_pointing_error_deg = sqrt(mean( ...
    pointing_error_deg.^2));

maximum_total_torque = ...
    max(vecnorm(total_torque_history, 2, 2));

maximum_gravity_torque = ...
    max(vecnorm(gravity_torque_history, 2, 2));

maximum_aerodynamic_torque = ...
    max(vecnorm(aerodynamic_torque_history, 2, 2));

maximum_srp_torque = ...
    max(vecnorm(srp_torque_history, 2, 2));

maximum_magnetic_torque = ...
    max(vecnorm(magnetic_torque_history, 2, 2));

maximum_wheel_speed_rpm = ...
    max(abs(wheel_speed_rpm), [], 1);

final_wheel_speed_rpm = ...
    wheel_speed_rpm(end,:);

%% Display results
fprintf('\nCombined disturbance-rejection results:\n');

fprintf('\nMaximum pointing error: %.6e deg\n', ...
    maximum_pointing_error_deg);

fprintf('RMS pointing error: %.6e deg\n', ...
    rms_pointing_error_deg);

fprintf('\nMaximum disturbance magnitudes [N m]:\n');
fprintf('Gravity gradient: %.6e\n', ...
    maximum_gravity_torque);
fprintf('Aerodynamic:      %.6e\n', ...
    maximum_aerodynamic_torque);
fprintf('SRP:              %.6e\n', ...
    maximum_srp_torque);
fprintf('Magnetic:         %.6e\n', ...
    maximum_magnetic_torque);
fprintf('Total:            %.6e\n', ...
    maximum_total_torque);

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

subplot(4,1,1);
plot(time, pointing_error_deg, ...
    'LineWidth', 1.2);
grid on;
ylabel('Pointing error [deg]');
title('Combined Environmental Disturbance Rejection');

subplot(4,1,2);
plot(time, total_torque_nNm, ...
    'LineWidth', 1.1);
grid on;
ylabel('Total torque [nN m]');
legend('\tau_x', '\tau_y', '\tau_z', ...
    'Location', 'best');

subplot(4,1,3);
plot(time, gravity_magnitude_nNm, ...
    'LineWidth', 1.1);
hold on;
plot(time, aerodynamic_magnitude_nNm, ...
    'LineWidth', 1.1);
plot(time, srp_magnitude_nNm, ...
    'LineWidth', 1.1);
plot(time, magnetic_magnitude_nNm, ...
    'LineWidth', 1.1);
grid on;
ylabel('Magnitude [nN m]');
legend('Gravity gradient', 'Aerodynamic', ...
    'SRP', 'Magnetic', 'Location', 'best');

subplot(4,1,4);
plot(time, wheel_speed_rpm, ...
    'LineWidth', 1.1);
grid on;
xlabel('Time [s]');
ylabel('Wheel speed [rpm]');
legend('\Omega_x', '\Omega_y', '\Omega_z', ...
    'Location', 'best');