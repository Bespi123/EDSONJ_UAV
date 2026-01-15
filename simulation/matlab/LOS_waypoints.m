clc; clear; close all;

% Parámetros de simulación
dt = 0.1;
T = 100;
steps = T / dt;

% Estado inicial del barco
x = 0; y = 0; psi = 0;
v = 1.5;  % velocidad propia (m/s)

% Corriente marina constante (en dirección X e Y)
v_cx = 0.2;  % corriente en X (m/s)
v_cy = 0.1;  % corriente en Y (m/s)

% Waypoints
waypoints = [10 10;
             25 0;
             40 20;
             60 0];

wp_index = 1;
lookahead = 5;  % distancia de anticipación (m)

% Historial para graficar
x_hist = zeros(1, steps);
y_hist = zeros(1, steps);

% Preparar figura
figure;
axis equal;
hold on;
grid on;
plot(waypoints(:,1), waypoints(:,2), 'ro--', 'LineWidth', 2);
xlabel('x (m)');
ylabel('y (m)');
title('Algoritmo LOS con corriente marina');

% Bucle de simulación
for k = 1:steps
    % Posición del siguiente waypoint
    x_wp = waypoints(wp_index, 1);
    y_wp = waypoints(wp_index, 2);
    
    % Distancia al waypoint
    dist_to_wp = sqrt((x_wp - x)^2 + (y_wp - y)^2);
    
    % Avanzar al siguiente waypoint si estamos cerca
    if dist_to_wp < 2
        if wp_index < size(waypoints, 1)
            wp_index = wp_index + 1;
        else
            break;
        end
    end
    
    % LOS con anticipación (lookahead point)
    dx = x_wp - x;
    dy = y_wp - y;
    heading_to_wp = atan2(dy, dx);
    
    % Punto de anticipación (lookahead)
    x_la = x + lookahead * cos(heading_to_wp);
    y_la = y + lookahead * sin(heading_to_wp);
    
    % Nuevo rumbo deseado
    psi_d = atan2(y_la - y, x_la - x);

    % Control de rumbo (proporcional)
    K_psi = 1.5;
    e_psi = wrapToPi(psi_d - psi);
    psi = psi + K_psi * e_psi * dt;
    
    % Velocidad total = propia + corriente
    x = x + (v * cos(psi) + v_cx) * dt;
    y = y + (v * sin(psi) + v_cy) * dt;

    % Guardar datos
    x_hist(k) = x;
    y_hist(k) = y;
    
    % Actualizar animación
    plot(x, y, 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
    drawnow;
end

% Trazo final
plot(x_hist, y_hist, 'b-', 'LineWidth', 1.5);
legend('Waypoints', 'Posición actual', 'Trayectoria');
