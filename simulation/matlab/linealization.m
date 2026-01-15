%% Model parameters
%%%EdsonJV Parameters
%%% Hydrodynamic and added mass parameters

% Initial conditions for velocities (in body frame)
u_0   = 2.0;   % surge velocity [m/s]
v_0   = eps;   % sway velocity [m/s]
r_0   = eps;   % yaw rate [rad/s]

% Initial conditions for actuators
n_c_0 = 2.0;   % Velocidad de propulsión de la hélice de estribor (RPS)
n_d_0 = 0.0;   % Velocidad de propulsión de la hélice de babor (RPS)

% Initial conditions for position and orientation (in global frame)
x_0     = 0;   % initial x-position [m]
y_0     = 0;   % initial y-position [m]
yaw_0   = 0;   % initial yaw angle [rad]

% Mass and moment of inertia
coef.m       = 250;          % kg - vehicle mass
coef.x_g     = 0.08;         % m - x-coordinate of the center of gravity
coef.I_z     = 204.1000;     % kg·m² - yaw moment of inertia

% Added mass coefficients
coef.X_u_dot = -2.4706;      % kg - added mass in surge
coef.Y_v_dot = -247.0649;    % kg - added mass in sway
coef.Y_r_dot = -370.5973;    % kg·m/rad - added mass coupling (sway/yaw)
coef.N_v_dot = -370.5973;    % kg·m - added mass coupling (yaw/sway)
coef.N_r_dot = -748.3102;    % kg·m²/rad - added mass in yaw

% Linear and nonlinear damping coefficients
coef.X_u     = -0.2912;      % kg/s - linear drag in surge
coef.X_uu    = -27.6262;     % kg/m - nonlinear drag in surge
coef.Y_v     = -123.5324;    % kg/s - linear drag in sway
coef.Y_vv    = -38.9275;     % kg/m - nonlinear drag in sway
coef.N_r     = -741.1947;    % kg·m²/s² - linear yaw damping
coef.N_rr    = -262.7909;    % kg·m² - nonlinear yaw damping

% Propeller-related coefficients (if needed)
coef.X_unc   = -3.6822;      % kg·m - propeller coefficient
coef.X_rnd   =  1.8411;      % kg·m - propeller coefficient
coef.X_ncnc  =  1.0518;      % kg·m² - propeller coefficient
coef.X_ndnd  =  1.0518;      % kg·m² - propeller coefficient
coef.Y_vnc   = -0.3588;      % kg - propeller coefficient
coef.Y_rnc   = -0.5382;      % kg·m - propeller coefficient
coef.N_und   =  2.7616;      % kg·m - propeller coefficient
coef.N_rnc   = -2.8549;      % kg·m² - propeller coefficient
coef.N_ncnd  = -1.5777;      % kg·m² - propeller coefficient
coef.N_vnc   = -0.5382;      % kg·m - propeller coefficient

%Llamar a la función de linealización completa
[A, B] = kinematic_jacobian(x_0, y_0, yaw_0, u_0, v_0, r_0, n_c_0, n_d_0, coef);

% Mostrar la matriz Jacobiana completa
disp('Matriz A:  x=[x,y,yaw,u,v,t]' );
disp(A);
disp('Matriz B:  u=[nc, nd]' );
disp(B);

% 1. Cálculo de los eigenvalores de la matriz A (para el análisis de estabilidad)
eigenvalues_A = eig(A);
disp('Eigenvalores de A:');
disp(eigenvalues_A);

% Comprobar estabilidad: si todos los eigenvalores tienen parte real negativa, el sistema es estable
if all(real(eigenvalues_A) < 0)
    disp('El sistema es asintóticamente estable.');
else
    disp('El sistema NO es asintóticamente estable.');
end

% 2. Cálculo de la matriz de controlabilidad y su rango
controllability_matrix = ctrb(A, B);
rank_controllability = rank(controllability_matrix);
disp('Rango de la matriz de controlabilidad:');
disp(rank_controllability);

% Verificar si el sistema es controlable
if rank_controllability == size(A, 1)
    disp('El sistema es controlable.');
else
    disp('El sistema NO es controlable.');
end
% -------------------------------
% Función para calcular la matriz Jacobiana de la cinemática
function [A_kin, B_kin] = kinematic_jacobian(x_0, y_0, yaw_0, u_0, v_0, r_0, n_c_0, n_d_0, coef)
    % Jacobiano de la función kinemática
    
    % Variables simbólicas para linealización
    syms x_sym y_sym u_sym v_sym r_sym yaw_sym n_c_sym n_d_sym
    
    % Extraer los coeficientes del sistema
    X_u_dot = coef.X_u_dot;
    Y_v_dot = coef.Y_v_dot;
    Y_r_dot = coef.Y_r_dot;
    x_g     = coef.x_g;
    N_v_dot = coef.N_v_dot;
    I_z     = coef.I_z;
    N_r_dot = coef.N_r_dot;
    
    % Vectores de velocidad
    v = [u_sym; v_sym; r_sym];
    
    % Matriz de transformación J(ψ)
    J = J_matrix(yaw_sym);
    
    % Kinematic equation
    n_dot = J*v;

    % Dynamic equation
    % Matriz de masa (incluso con masas añadidas)
    M = [coef.m - X_u_dot,           0,                      0;
              0,      coef.m - Y_v_dot,       coef.m * x_g - Y_r_dot;
              0,      coef.m * x_g - N_v_dot,     I_z - N_r_dot];
          
    % Matriz de Coriolis y centrípeta (con términos de masa añadida)
    C = [0,          -coef.m * r_sym,         -coef.m * x_g * r_sym + Y_v_dot * v_sym + Y_r_dot * r_sym;
         coef.m * r_sym,         0,           -X_u_dot * u_sym;
         coef.m * x_g * r_sym - Y_v_dot * v_sym - Y_r_dot * r_sym, X_u_dot * u_sym, 0];
     
    % Matriz de amortiguamiento (lineales y no lineales)
    D = - [coef.X_u + coef.X_uu * abs(u_sym),              0,                   0;
                     0,     coef.Y_v + coef.Y_vv * abs(v_sym),               0;
                     0,                 0,      coef.N_r + coef.N_rr * abs(r_sym)];
 
    % 1. Definir los vectores de entrada de control lineal y no lineal
    u_lin = [n_c_sym; ...
             n_d_sym];
    
    u_nonlin = [n_c_sym^2; ...
                n_d_sym^2; ...
                n_c_sym * n_d_sym];
    
    % 2. Definir la matriz B_L (dependiente del estado u, v_es, r)
    B_L = [coef.X_unc*u_sym,                             coef.X_rnd*r_sym; ...
           (coef.Y_vnc*v_sym + coef.Y_rnc*r_sym),              0; ...
           (coef.N_vnc*v_sym + coef.N_rnc*r_sym),         coef.N_und*u_sym];
    
    % 3. Definir la matriz B_NL (constante para este modelo)
    B_NL = [coef.X_ncnc,  coef.X_ndnd,  0; ...
            0,       0,       0; ...
            0,       0,       coef.N_ncnd];

    % 4. Calcular tao usando la forma factorizada
    tao = B_L * u_lin + B_NL * u_nonlin;

    v_dot = M \(-C*v - D*v + tao);
    
    x_dot = [n_dot; v_dot];

    % Calcular el Jacobiano de η̇ = J * ν con respecto a los estados y entradas
    A_kin = jacobian(x_dot, [x_sym, y_sym, yaw_sym, u_sym, v_sym, r_sym]);
    
    % Evaluar el Jacobiano en el punto de operación
    A_kin = double(subs(A_kin, {x_sym, y_sym, yaw_sym, u_sym, v_sym, r_sym, n_c_sym, n_d_sym}, ...
                              {x_0, y_0, yaw_0, u_0, v_0, r_0, n_c_0, n_d_0}));

    B_kin = jacobian(x_dot, [n_c_sym, n_d_sym]);
    % Evaluar el Jacobiano en el punto de operación
    B_kin = double(subs(B_kin, {x_sym, y_sym, yaw_sym, u_sym, v_sym, r_sym, n_c_sym, n_d_sym}, ...
                              {x_0, y_0, yaw_0, u_0, v_0, r_0, n_c_0, n_d_0}));
end

function J = J_matrix(yaw)
    % Construct the rotation matrix J(ψ)
    J = [cos(yaw), -sin(yaw), 0;
         sin(yaw),  cos(yaw), 0;
               0,        0,  1];
end