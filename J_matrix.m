function b_arr = J_matrix(input)
% J_matrix.m
%
% This function computes the 2D rotation matrix J(ψ) used in the kinematic
% transformation of a marine vehicle (e.g., USV), based on the yaw angle ψ.
%
% INPUT:
%   input : a 3-element vector η = [x; y; ψ]
%           ψ : yaw angle (orientation of the vehicle in global frame)
%
% OUTPUT:
%   b_arr : a 1×9 row vector containing the elements of the rotation matrix J(ψ),
%           ordered row-wise, i.e., b_arr = reshape(J', 1, [])
%
%           J(ψ) =
%           [cos(ψ), -sin(ψ), 0;
%            sin(ψ),  cos(ψ), 0;
%                 0,       0, 1]


% Extract yaw angle from the input vector η
    n = [input(1), input(2), input(3)]';  % η = [x; y; ψ]
    yaw = n(3);                           % ψ (yaw angle)

    % Construct the rotation matrix J(ψ)
    b = [cos(yaw), -sin(yaw), 0;
         sin(yaw),  cos(yaw), 0;
               0,        0,  1];

    % Reshape the matrix into a 1x9 row vector (row-wise)
    b_arr = reshape(b', 1, []);  % Transpose and flatten
end