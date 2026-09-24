function quaternion = euler_to_quaternion(euler_angles)
%EULER_TO_QUATERNION Convert 3-2-1 Euler angles to a quaternion.
%
% Input:
%   euler_angles = [roll; pitch; yaw] in rad
%
% Output:
%   quaternion = [qw; qx; qy; qz], scalar first
%
% The quaternion represents the body orientation relative to
% the inertial reference frame.

roll  = euler_angles(1);
pitch = euler_angles(2);
yaw   = euler_angles(3);

cr = cos(roll / 2);
sr = sin(roll / 2);

cp = cos(pitch / 2);
sp = sin(pitch / 2);

cy = cos(yaw / 2);
sy = sin(yaw / 2);

qw = cr*cp*cy + sr*sp*sy;
qx = sr*cp*cy - cr*sp*sy;
qy = cr*sp*cy + sr*cp*sy;
qz = cr*cp*sy - sr*sp*cy;

quaternion = [qw; qx; qy; qz];

% Protect against numerical roundoff.
quaternion = quaternion / norm(quaternion);

end