function quaternion = ...
    rotation_vector_to_quaternion(rotation_vector)
%ROTATION_VECTOR_TO_QUATERNION Convert a rotation vector to a quaternion.
%
% Input:
%   rotation_vector - axis multiplied by angle, in rad
%
% Output:
%   quaternion      - [qw; qx; qy; qz], scalar first

rotation_angle = norm(rotation_vector);

if rotation_angle < 1e-12
    % Small-angle approximation
    quaternion = [
        1;
        0.5 * rotation_vector
    ];
else
    rotation_axis = ...
        rotation_vector / rotation_angle;

    quaternion = [
        cos(rotation_angle / 2);
        rotation_axis * sin(rotation_angle / 2)
    ];
end

quaternion = quaternion / norm(quaternion);

end