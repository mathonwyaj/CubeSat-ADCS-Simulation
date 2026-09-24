function euler_dot = euler_kinematics(euler_angles, omega)
%EULER_KINEMATICS Convert body rates to Euler-angle rates.
%
% Uses the 3-2-1 yaw-pitch-roll convention.
%
% Inputs:
%   euler_angles - [roll; pitch; yaw] in rad
%   omega        - body rates [p; q; r] in rad/s
%
% Output:
%   euler_dot    - [roll_rate; pitch_rate; yaw_rate] in rad/s

phi   = euler_angles(1);
theta = euler_angles(2);

transformation = [
    1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
    0, cos(phi),            -sin(phi);
    0, sin(phi)/cos(theta),  cos(phi)/cos(theta)
];

euler_dot = transformation * omega;

end