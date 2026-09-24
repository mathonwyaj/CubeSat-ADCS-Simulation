function omega_dot = rigid_body_dynamics(omega, tau, I)
%RIGID_BODY_DYNAMICS Calculate three-axis angular acceleration.
%
% omega: body angular velocity [p; q; r] in rad/s
% tau:   applied body torque [tau_x; tau_y; tau_z] in N m
% I:     spacecraft inertia matrix in kg m^2

angular_momentum = I * omega;

gyroscopic_torque = cross(omega, angular_momentum);

omega_dot = I \ (tau - gyroscopic_torque);

end