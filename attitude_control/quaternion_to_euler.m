function euler_angles = quaternion_to_euler(quaternion)
%QUATERNION_TO_EULER Convert a quaternion to 3-2-1 Euler angles.
%
% Input:
%   quaternion = [qw; qx; qy; qz], scalar first
%
% Output:
%   euler_angles = [roll; pitch; yaw] in rad

quaternion = quaternion / norm(quaternion);

qw = quaternion(1);
qx = quaternion(2);
qy = quaternion(3);
qz = quaternion(4);

%% Roll
roll = atan2( ...
    2 * (qw*qx + qy*qz), ...
    1 - 2 * (qx^2 + qy^2));

%% Pitch
pitch_argument = 2 * (qw*qy - qz*qx);

% Protect asin from small numerical excursions outside [-1, 1].
pitch_argument = max(-1, min(1, pitch_argument));

pitch = asin(pitch_argument);

%% Yaw
yaw = atan2( ...
    2 * (qw*qz + qx*qy), ...
    1 - 2 * (qy^2 + qz^2));

euler_angles = [roll; pitch; yaw];

end