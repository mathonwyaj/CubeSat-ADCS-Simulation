function state_dot = closed_loop_dynamics_3axis( ...
    time, state, attitude_cmd, Kp, Kd, tau_max, I)
%CLOSED_LOOP_DYNAMICS_3AXIS Controller and spacecraft dynamics.
%
% State:
%   state(1:3) = Euler angles [roll; pitch; yaw] in rad
%   state(4:6) = body rates [p; q; r] in rad/s

attitude = state(1:3);
omega = state(4:6);

% Calculate the saturated control torque.
tau = pd_controller_3axis( ...
    attitude_cmd, attitude, omega, Kp, Kd, tau_max);

% Apply that torque to the spacecraft.
state_dot = spacecraft_dynamics_3axis(time, state, tau, I);

end