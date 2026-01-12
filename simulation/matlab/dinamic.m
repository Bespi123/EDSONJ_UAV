function v_dot = dinamic(input)
% DINAMIC Enhanced 3-DOF USV dynamics with coupled propulsion effects
%
% Computes body-fixed accelerations for a USV with twin-propeller system
% using Newton-Euler equations with hydrodynamic coupling terms
%
% INPUT:
%   input : 1×29 vector containing:
%     Propulsion:
%       input(1:2)   = [n_c, n_d]       : Starboard/port propeller speeds [RPS]
%     Velocities:
%       input(3:5)   = [u; v; r]        : Body-fixed velocities [m/s, m/s, rad/s]
%     Physical Parameters:
%       input(6)     = m                 : Mass [kg]
%       input(7)     = X_u_dot           : Added mass in surge [kg]
%       input(8)     = Y_v_dot           : Added mass in sway [kg]
%       input(9)     = Y_r_dot           : Added mass coupling [kg·m]
%       input(10)    = x_g               : CG x-coordinate [m]
%       input(11)    = N_v_dot           : Added mass coupling [kg·m]
%       input(12)    = I_z               : Yaw inertia [kg·m²]
%       input(13)    = N_r_dot           : Added mass in yaw [kg·m²]
%     Damping Coefficients:
%       input(14)    = X_u               : Linear surge damping [kg/s]
%       input(15)    = X_uu              : Quadratic surge damping [kg/m]
%       input(16)    = Y_v               : Linear sway damping [kg/s]
%       input(17)    = Y_vv              : Quadratic sway damping [kg/m]
%       input(18)    = N_r               : Linear yaw damping [kg·m²/s]
%       input(19)    = N_rr              : Quadratic yaw damping [kg·m²]
%     Propulsion Coupling Coefficients:
%       input(20:29) : Hydrodynamic propulsion terms (see below)
%
% OUTPUT:
%   v_dot : 3×1 vector of accelerations [u̇; v̇; ṙ] [m/s², m/s², rad/s²]

    % Extract forces/moments and velocities
    n_c = input(1);     % Starboard propeller speed [RPS]
    n_d = input(2);     % Port propeller speed [RPS]
    v = [input(3); input(4); input(5)];    % Velocity vector [u; v; r]

    u = v(1);           % Surge velocity [m/s]
    v_es = v(2);        % Sway velocity [m/s]
    r = v(3);           % Yaw rate [rad/s]

    % Extract physical parameters and hydrodynamic coefficients
    m       = input(6);
    X_u_dot = input(7);
    Y_v_dot = input(8);
    Y_r_dot = input(9);
    x_g     = input(10);
    N_v_dot = input(11);
    I_z     = input(12);
    N_r_dot = input(13);

    X_u  = input(14);
    X_uu = input(15);
    Y_v  = input(16);
    Y_vv = input(17);
    N_r  = input(18);
    N_rr = input(19);

    X_unc  = input(20);
    X_rnd  = input(21);
    X_ncnc = input(22);
    X_ndnd = input(23);
    Y_vnc  = input(24);
    Y_rnv  = input(25);
    N_vnc  = input(26);
    N_rnc  = input(27);
    N_und  = input(28);
    N_ncnd = input(29);

    % Inertia matrix (including added mass) 
    % Eq. 7.16 asumming N_v_dot =  Y_r_dot
    M = [m - X_u_dot,           0,                      0;
              0,      m - Y_v_dot,       m * x_g - Y_r_dot;
              0,      m * x_g - N_v_dot,     I_z - N_r_dot];

    % Coriolis-centripetal matrix (with added mass terms)
    % Eq. 6.7
    % Eq. 6.51
    C = [0,          -m*r,         -m*x_g*r + Y_v_dot*v_es + Y_r_dot*r;
         m*r,         0,           -X_u_dot*u;
         m*x_g*r - Y_v_dot*v_es - Y_r_dot*r, X_u_dot*u, 0];

    % Damping matrix (linear + nonlinear hydrodynamic damping)
    % Eq. 7.19 - 7.24
    % Eq. 7.19 considering N_v = 0 and Y_r = 0 
    % Eq. 7.24 simplificated for larger ships N_vv = 0 and Y_vr = 0 
    D = - [X_u + X_uu * abs(u),              0,                   0;
                     0,     Y_v + Y_vv * abs(v_es),               0;
                     0,                 0,      N_r + N_rr * abs(r)];

    % Propeller Wageningen model
    tao = [X_unc*u*n_c + X_rnd*r*n_d + X_ncnc*n_c^2 + X_ndnd*n_d^2; ... 
           Y_vnc*v_es*n_c + Y_rnv*r*n_c; ...
           N_vnc*v_es*n_c + N_rnc*r*n_c + N_und*u*n_d+ N_ncnd*n_c*n_d];

    % Compute the acceleration: ν̇ = M⁻¹ (τ - Cν - Dν)
    v_dot = M \ (-C*v - D*v + tao);
end
