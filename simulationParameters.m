%%%EdsonJV Parameters
%%% Hydrodynamic and added mass parameters

% Initial conditions for velocities (in body frame)
u_0   = 0;   % surge velocity [m/s]
v_0   = 0;   % sway velocity [m/s]
r_0   = 0;   % yaw rate [rad/s]

% Initial conditions for position and orientation (in global frame)
x_0     = 0;   % initial x-position [m]
y_0     = 0;   % initial y-position [m]
yaw_0   = 0;   % initial yaw angle [rad]

% Mass and moment of inertia
m       = 250;          % kg - vehicle mass
x_g     = 0.08;         % m - x-coordinate of the center of gravity
I_z     = 204.1000;     % kg·m² - yaw moment of inertia

% Added mass coefficients
X_u_dot = -2.4706;      % kg - added mass in surge
Y_v_dot = -247.0649;    % kg - added mass in sway
Y_r_dot = -370.5973;    % kg·m/rad - added mass coupling (sway/yaw)
N_v_dot = -370.5973;    % kg·m - added mass coupling (yaw/sway)
N_r_dot = -748.3102;    % kg·m²/rad - added mass in yaw

% Linear and nonlinear damping coefficients
X_u     = -0.2912;      % kg/s - linear drag in surge
X_uu    = -27.6262;     % kg/m - nonlinear drag in surge
Y_v     = -123.5324;    % kg/s - linear drag in sway
Y_vv    = -38.9275;     % kg/m - nonlinear drag in sway
N_r     = -741.1947;    % kg·m²/s² - linear yaw damping
N_rr    = -262.7909;    % kg·m² - nonlinear yaw damping

% Propeller-related coefficients (if needed)
X_unc   = -3.6822;      % kg·m - propeller coefficient
X_rnd   =  1.8411;      % kg·m - propeller coefficient
X_ncnc  =  1.0518;      % kg·m² - propeller coefficient
X_ndnd  =  1.0518;      % kg·m² - propeller coefficient
Y_vnc   = -0.3588;      % kg - propeller coefficient
Y_rnc   = -0.5382;      % kg·m - propeller coefficient
N_und   =  2.7616;      % kg·m - propeller coefficient
N_rnc   = -2.8549;      % kg·m² - propeller coefficient
N_ncnd  = -1.5777;      % kg·m² - propeller coefficient
N_vnc   = -0.5382;      % kg·m - propeller coefficient

% % % % M = [m-X_u_dot,         0,                 0;
% % % %              0, m-Y_v_dot,     m*x_g-Y_r_dot;
% % % %              0, m*x_g-N_v_dot,   I_z-N_r_dot];
% % % % 
% % % % M = reshape(M', 1, []);  % Transpone y reorganiza como vector fila
% % % % 
% % % % % C = [        0,      -m, -m*x_g+Y_v_dot*+Y_r_dot;
% % % % %              m,       0, -X_u_dot;
% % % % %            m*x_g-Y_v_dot-Y_r_dot, X_u_dot, 0];
% % % % % C = reshape(C', 1, []);  % Transpone y reorganiza como vector fila
% % % % 
% % % % D = [X_u+X_uu,                  0, 0;
% % % %                    0, Y_v+Y_vv, 0;
% % % %                    0,                  0, N_r+N_rr];
% % % % D = reshape(D', 1, []);  % Transpone y reorganiza como vector fila
% % % 
% % % 
% % % M = [ 1.1274   0        0;
% % %       0        1.8902  -0.0744;
% % %       0       -0.0744   0.1287 ];
% % % M = reshape(M', 1, []);  % Transpone y reorganiza como vector fila
% % % 
% % % D = [ 0.0414   0        0;
% % %       0        0.1775  -0.0141;
% % %       0       -0.01073  0.0568 ];
% % %  D = reshape(D', 1, []);  % Transpone y reorganiza como vector fila
% % % 
% % % k1 = 0.2;
% % % c1 = 0.9;
% % % p  = 9;
% % % q  = 11;
% % % phi1 = 0.2;
% % % h1   = 0.3;
% % % h2   = 0.2;
% % % lambda1 = 0.5;
% % % lambda2 = 1.5;
% % % h3      = 0.3;
% % % gamma   = 0.1;
% % % mu = 0.001;
% % % set_point = [25,30,15*pi/180];
