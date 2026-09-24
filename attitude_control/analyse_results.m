%% CubeSat Attitude Control - Results Analysis

% Extract simulation outputs
theta = out.theta_out;
omega = out.omega_out;
torque = out.torque_out;

% Convert attitude and angular velocity from radians to degrees
theta_deg = rad2deg(theta.Data);
omega_deg_s = rad2deg(omega.Data);
t = theta.Time;

% Calculate performance metrics
final_error = abs(theta_deg(end));
max_omega = max(abs(omega_deg_s));
max_torque = max(abs(torque.Data));

% Calculate settling time within +/- 0.5 degrees
tolerance = 0.5;
settling_time = NaN;

for i = 1:length(theta_deg)
    if all(abs(theta_deg(i:end)) <= tolerance)
        settling_time = t(i);
        break
    end
end

% Display results
fprintf('\n--- CubeSat Attitude Control Results ---\n');
fprintf('Final pointing error: %.4f deg\n', final_error);
fprintf('Maximum angular velocity: %.4f deg/s\n', max_omega);
fprintf('Maximum control torque: %.6f N*m\n', max_torque);
fprintf('Settling time (+/- %.1f deg): %.3f s\n', ...
    tolerance, settling_time);

% Plot attitude response
figure;
plot(t, theta_deg, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Attitude Angle (deg)');
title('CubeSat Attitude Control Response');
yline(0, '--');
yline(tolerance, ':');
yline(-tolerance, ':');

% Plot angular velocity
figure;
plot(omega.Time, omega_deg_s, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
title('CubeSat Angular Velocity Response');
yline(0, '--');

% Plot actuator torque
figure;
plot(torque.Time, torque.Data, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Control Torque (N m)');
title('CubeSat Actuator Torque');
yline(tau_max, ':');
yline(-tau_max, ':');

% Save figures to results folder
results_folder = fullfile('..', 'results');

if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

figure(1);
exportgraphics(gcf, fullfile(results_folder, 'attitude_response.png'), ...
    'Resolution', 300);

figure(2);
exportgraphics(gcf, fullfile(results_folder, 'angular_velocity_response.png'), ...
    'Resolution', 300);

figure(3);
exportgraphics(gcf, fullfile(results_folder, 'actuator_torque_response.png'), ...
    'Resolution', 300);