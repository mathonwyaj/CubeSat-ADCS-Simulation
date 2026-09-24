function quaternion_dot = quaternion_kinematics(quaternion, omega)
%QUATERNION_KINEMATICS Calculate the quaternion derivative.
%
% Inputs:
%   quaternion = [qw; qx; qy; qz], scalar first
%   omega      = body rates [p; q; r] in rad/s
%
% Output:
%   quaternion_dot = time derivative of the quaternion

% Normalize the input to protect against accumulated numerical error.
quaternion = quaternion / norm(quaternion);

angular_velocity_quaternion = [0; omega];

quaternion_dot = 0.5 * quaternion_multiply( ...
    quaternion, angular_velocity_quaternion);

end