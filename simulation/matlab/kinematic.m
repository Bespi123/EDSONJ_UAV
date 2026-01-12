function n_dot = kinematic(input)
% kinematic.m
%
% This function computes the time derivative of the position and orientation
% vector (η̇) of a marine vehicle (e.g., USV) using the kinematic transformation:
% η̇ = J(ψ) * ν
%
% INPUT:
%   input : a 12-element vector where:
%       input(1:3)   = ν = [u; v; r], the body-fixed velocity vector:
%                     u : surge velocity (forward)
%                     v : sway velocity (sideways)
%                     r : yaw rate (angular velocity)
%       input(4:12)  = elements of the 3×3 rotation matrix J(ψ) flattened row-wise:
%                     J = [cos(ψ), -sin(ψ), 0;
%                          sin(ψ),  cos(ψ), 0;
%                              0,       0, 1]
%
% OUTPUT:
%   n_dot : 3×1 vector representing η̇ = [ẋ; ẏ; ψ̇], the rate of change of the global pose

    % Extract body-fixed velocity vector ν
    v = [input(1), input(2), input(3)]';

    % Reconstruct the transformation matrix J(ψ)
    J = [input(4), input(5), input(6);
         input(7), input(8), input(9);
         input(10), input(11), input(12)]; 

    % Compute η̇ = J * ν
    n_dot = J * v;
end
