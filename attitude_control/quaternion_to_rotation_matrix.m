function rotation_matrix = ...
    quaternion_to_rotation_matrix(quaternion)
%QUATERNION_TO_ROTATION_MATRIX
% Convert a scalar-first quaternion into a body-to-inertial
% direction-cosine matrix.
%
% Input:
%   quaternion = [qw; qx; qy; qz]
%
% Output:
%   rotation_matrix transforms a body-frame vector into
%   inertial-frame coordinates.

quaternion = quaternion / norm(quaternion);

qw = quaternion(1);
qx = quaternion(2);
qy = quaternion(3);
qz = quaternion(4);

rotation_matrix = [
    1 - 2*(qy^2 + qz^2), ...
    2*(qx*qy - qw*qz), ...
    2*(qx*qz + qw*qy);

    2*(qx*qy + qw*qz), ...
    1 - 2*(qx^2 + qz^2), ...
    2*(qy*qz - qw*qx);

    2*(qx*qz - qw*qy), ...
    2*(qy*qz + qw*qx), ...
    1 - 2*(qx^2 + qy^2)
];

end