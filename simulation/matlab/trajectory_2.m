clear; clc; close all;
    
% Lineal model
linealization

% 1. Define Path
waypoints_x = 1+2*[0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120]; % East
waypoints_y = 1+2*[0,  5, 2,  5,  0, 10, 5, 0,  5, 2,  5,  0, 10];  % North

% Parameterize Path by Arc Length (theta = s)
[path_x_fn, path_y_fn, deriv_x_fn, deriv_y_fn, total_s, x_f, y_f] = ...
    parameterize_path_by_arc_length(waypoints_x, waypoints_y);

% Store path info
path_params.path_x_fn  = path_x_fn;
path_params.path_y_fn  = path_y_fn;
path_params.deriv_x_fn = deriv_x_fn; % Corresponds to getting xp'(theta)
path_params.deriv_y_fn = deriv_y_fn; % Corresponds to getting yp'(theta)
path_params.total_s = total_s;

% Controller values
kp_u = -14979.4861424611;
ki_u = -519568.568936889; 
kd_u = -14.2966203625565;
N_u  = 715.765273164083;

kp_r = 503.874847376787;
ki_r = 651.327515048563; 
kd_r = 48.6562327982834;
N_r  = 8.68555609659204;

% 2. Simulation Setup
time_max = 120; % Max simulation time
dt = 0.001;
t = 0: dt: time_max; 
vehicle_speed_u = 2.0; % Constant surge velocity 'u' (m/s)

% Initial state: [x(East); y(North); psi(Yaw); velocity_u; velocity_v; velocity_r]
initial_state = [0; 0; 0; vehicle_speed_u; 0; 0]; % Initial position and velocity
y = initial_state; % Store initial state in vector y

% Lookahead distance (Delta) based on vehicle speed
%delta_lookahead = vehicle_speed_u * 3; 
delta_lookahead = vehicle_speed_u; 
%delta_lookahead = 1;
%%
% Persistent variable for theta guess
theta_persistent_guess = 0;

% Initial values for PID integral and derivative terms
int_u = 0;
int_yaw = 0;
int_u_ant = 0;
int_yaw_ant = 0;

prev_u_e = 0;
prev_yaw_e = 0;

chi_d_ant = 0;

yf_u = 0; % Derivative filter for u
yf_y = 0; % Derivative filter for yaw

% --- Contenedores para almacenar las señales de control (u) y el estado (y) ---
u_container = zeros(2, length(t)); % Almacena las señales de control (surge y yaw)
y_container = zeros(6, length(t)); % Almacena el estado (x, y, yaw, u, v, r)

% --- Run Simulation ---
for i=1:length(t)
    
    % Extract vehicle state variables
    veh_x   = y(1);
    veh_y   = y(2);
    veh_yaw = y(3);
    veh_u   = y(4);
    veh_v   = y(5);
    veh_r   = y(6);
    
    % --- Guidance Law ---
    [chi_d, ~, ~, theta_star] = los_guidance(veh_x, veh_y, path_params, delta_lookahead, theta_persistent_guess);
    theta_persistent_guess = theta_star; % Update persistent guess
    chi_d_dot = (chi_d - chi_d_ant)/dt;

    % Error in velocity and yaw
    u_e   = vehicle_speed_u - veh_u;
    yaw_e = wrapToPi(chi_d_dot - veh_r);
    %yaw_e = wrapToPi(0 - veh_yaw);
        
    % PID controller
    % 2. Update integral terms
    int_u   = int_u_ant   + u_e * dt;
    int_yaw = int_yaw_ant + yaw_e * dt;

    % 3. Calculate derivatives of the error (finite differences)
    u_e_dot   = (u_e   - prev_u_e)/dt;
    yaw_e_dot = (yaw_e - prev_yaw_e)/dt;

    % 4. Implement derivative filter with Runge-Kutta (4th order)
    % For surge velocity:
    k1 = N_u * (u_e_dot - yf_u);
    k2 = N_u * (u_e_dot - (yf_u + k1*dt/2));
    k3 = N_u * (u_e_dot - (yf_u + k2*dt/2));
    k4 = N_u * (u_e_dot - (yf_u + k3*dt));
    yf_u = yf_u + (k1 + 2*k2 + 2*k3 + k4)*dt/6;

    % For yaw velocity:
    k1 = N_r * (yaw_e_dot - yf_y);
    k2 = N_r * (yaw_e_dot - (yf_y + k1*dt/2));
    k3 = N_r * (yaw_e_dot - (yf_y + k2*dt/2));
    k4 = N_r * (yaw_e_dot - (yf_y + k3*dt));
    yf_y = yf_y + (k1 + 2*k2 + 2*k3 + k4)*dt/6;

    % 5. Calculate control signals
    u_surge = kp_u*u_e + ki_u*int_u + kd_u*yf_u;
    u_yaw   = kp_r*yaw_e + ki_r*int_yaw + kd_r*yf_y;

    u = [u_surge; u_yaw];
    
    % Store control signals in container
    u_container(:, i) = u;

    % Runge-Kutta 4th order integration for dynamics
    k1 = edsonj_dynamics(t(i), y, u, A, B);
    k2 = edsonj_dynamics(t(i) + dt/2, y + dt/2*k1, u, A, B);
    k3 = edsonj_dynamics(t(i) + dt/2, y + dt/2*k2, u, A, B);
    k4 = edsonj_dynamics(t(i) + dt, y + dt*k3, u, A, B);
    
    % Update state
    y = y + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    % Store state in container
    y_container(:, i) = y;

    % Update previous integral and error values
    int_u_ant = int_u;
    int_yaw_ant = int_yaw; 
    prev_u_e = u_e;
    prev_yaw_e = yaw_e;

    chi_d_ant = chi_d;
end

figure;
plot(waypoints_x, waypoints_y, '.b','MarkerSize',20); hold on
plot(x_f, y_f, '.r', 'LineWidth', 2);
plot(y_container(1,:), y_container(2,:), '--g', 'LineWidth', 2);
% Etiquetas y título
xlabel('East (X)');
ylabel('North (Y)');
legend('Waypoints','Referencia (Camino parametrizado)', 'Camino actual del vehículo');
title('Referencia vs Camino Actual');

figure;
subplot(3,1,1);
plot(t, u_container(1,:), '-r', 'LineWidth', 2);
hold on;
plot(t, u_container(2,:), '-b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Control Signals');
legend('Surge Control (u)', 'Yaw Control (u_yaw)');
title('Control Signals Over Time');
grid on;

subplot(3,1,2);
plot(t, y_container(1,:), '-r', 'LineWidth', 2);
hold on;
plot(t, y_container(2,:), '-g', 'LineWidth', 2);
plot(t, y_container(3,:), '-b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('State Variables');
legend('x (East)', 'y (North)', 'yaw (psi)');
title('Vehicle States Over Time');
grid on;

subplot(3,1,3);
plot(t, y_container(4,:), '-r', 'LineWidth', 2);
hold on;
plot(t, y_container(5,:), '-g', 'LineWidth', 2);
plot(t, y_container(6,:), '-b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('State Variables');
legend('p', 'q', 'r');
title('Vehicle States Over Time');
grid on;


% 1. parameterize_path_by_arc_length
function [path_x_spline, path_y_spline, deriv_x_spline, deriv_y_spline, total_s, x_fine, y_fine] = ...
    parameterize_path_by_arc_length(wp_x, wp_y)
    % Parameterizes the path defined by waypoints (wp_x, wp_y) by arc length s.
    % Returns spline functions (ppform) for x(s), y(s), dx/ds, dy/ds, and total length.

    % Increment th number o
    num_initial_points = length(wp_x) * 50;
    x_fine = linspace(min(wp_x), max(wp_x), num_initial_points);
    y_fine = spline(wp_x, wp_y, x_fine);

    dx = diff(x_fine);
    dy = diff(y_fine);
    segment_lengths = sqrt(dx.^2 + dy.^2);
    s_cumulative = [0, cumsum(segment_lengths)];
    total_s = s_cumulative(end);

    [s_unique, ia, ~] = unique(s_cumulative, 'stable');
    x_for_s_spline = x_fine(ia);
    y_for_s_spline = y_fine(ia);
    
    if length(s_unique) < 2
        error('Not enough unique points to create arc-length parameterized spline.');
    end

    path_x_spline = spline(s_unique, x_for_s_spline); % x_p(theta) where theta=s
    path_y_spline = spline(s_unique, y_for_s_spline); % y_p(theta) where theta=s

    % Get derivatives dx/dtheta and dy/dtheta (needed for Eq. 1)
    deriv_x_spline = fnder(path_x_spline, 1); % xp'(theta)
    deriv_y_spline = fnder(path_y_spline, 1); % yp'(theta)
end


% 2. find_closest_point_on_path
function [theta_star, x_p, y_p, idx_closest] = find_closest_point_on_path(veh_x, veh_y, path_x_fn, path_y_fn, total_s, num_search_points)
    % Finds closest point theta_star by simple search.
    % Conceptually related to finding theta satisfying Eq. (3) or Def. 1.
    if nargin < 6
        num_search_points = 500;
    end
    theta_search = linspace(0, total_s, num_search_points);
    path_points_x = ppval(path_x_fn, theta_search);
    path_points_y = ppval(path_y_fn, theta_search);
    distances_sq = (path_points_x - veh_x).^2 + (path_points_y - veh_y).^2;
    [~, idx_closest] = min(distances_sq);
    theta_star = theta_search(idx_closest);
    x_p = path_points_x(idx_closest);
    y_p = path_points_y(idx_closest);
end

% 3. los_guidance
function [chi_d, y_e, gamma_p, theta_star] = los_guidance(veh_x, veh_y, path_params, delta_lookahead, current_theta_guess)
    % Calculates LOS guidance outputs based on Fossen & Pettersen paper.

    path_x_fn  = path_params.path_x_fn;
    path_y_fn  = path_params.path_y_fn;
    deriv_x_fn = path_params.deriv_x_fn;
    deriv_y_fn = path_params.deriv_y_fn;
    total_s = path_params.total_s;

    % Find projection point theta_star (conceptually related to Eq. 3 / Def. 1)
    [theta_star, x_p, y_p, ~] = find_closest_point_on_path(veh_x, veh_y, path_x_fn, path_y_fn, total_s);

    % Calculate path tangential angle gamma_p at theta_star
    dx_dtheta = ppval(deriv_x_fn, theta_star);
    dy_dtheta = ppval(deriv_y_fn, theta_star);
    gamma_p   = atan2(dy_dtheta, dx_dtheta); % Implements Eq. (1)

    % Calculate signed cross-track error y_e
    % Derived from coordinate transformation Eq. (2)
    y_e = -(veh_x - x_p) * sin(gamma_p) + (veh_y - y_p) * cos(gamma_p); % Implements Eq. (4)

    % Calculate desired course chi_d using LOS law
    chi_d = gamma_p + atan2(-y_e, delta_lookahead); % Implements Eq. (16)

    % Normalize angle
    chi_d = wrapToPi(chi_d);
end

% Helper function for angle wrapping
function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

function x_dot = edsonj_dynamics(t, x, u, A, B)
    x_dot = A*x+B*u;
end