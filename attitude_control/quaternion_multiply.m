function product = quaternion_multiply(q1, q2)
%QUATERNION_MULTIPLY Multiply two scalar-first quaternions.
%
% Inputs:
%   q1 = [qw; qx; qy; qz]
%   q2 = [qw; qx; qy; qz]
%
% Output:
%   product = q1 multiplied by q2

scalar1 = q1(1);
vector1 = q1(2:4);

scalar2 = q2(1);
vector2 = q2(2:4);

product_scalar = ...
    scalar1 * scalar2 - dot(vector1, vector2);

product_vector = ...
    scalar1 * vector2 + ...
    scalar2 * vector1 + ...
    cross(vector1, vector2);

product = [product_scalar; product_vector];

end