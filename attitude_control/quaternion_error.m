function error_quaternion = ...
    quaternion_error(command_quaternion, current_quaternion)
%QUATERNION_ERROR Calculate body-frame attitude error.
%
% Inputs:
%   command_quaternion - desired attitude [qw; qx; qy; qz]
%   current_quaternion - current attitude [qw; qx; qy; qz]
%
% Output:
%   error_quaternion   - attitude error [qw; qx; qy; qz]

command_quaternion = ...
    command_quaternion / norm(command_quaternion);

current_quaternion = ...
    current_quaternion / norm(current_quaternion);

current_conjugate = [
    current_quaternion(1);
   -current_quaternion(2:4)
];

error_quaternion = quaternion_multiply( ...
    current_conjugate, command_quaternion);

error_quaternion = ...
    error_quaternion / norm(error_quaternion);

% q and -q represent the same attitude.
% Choose the representation corresponding to the shorter rotation.
if error_quaternion(1) < 0
    error_quaternion = -error_quaternion;
end

end