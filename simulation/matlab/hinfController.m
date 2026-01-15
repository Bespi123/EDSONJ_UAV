% -------------------------------------------------------------------------
% Diseño de un Controlador de Seguimiento H-Infinito (H∞) con Mixsyn
% -------------------------------------------------------------------------
% Este script utiliza el método de síntesis de sensibilidad mixta (mixsyn)
% para diseñar un controlador H∞ de seguimiento. Este enfoque es
% numéricamente más robusto que la construcción manual de la planta.
%
% El objetivo es que las velocidades 'u' y 'r' sigan una referencia.
% -------------------------------------------------------------------------

clear; clc; close all;

%% 1. Definición del Modelo de Espacio de Estados del Vehículo
% Se ejecuta el script de linealización para obtener las matrices A y B.
% Se asume que 'linealization.m' está en el mismo directorio.
linealization;

% Seleccionamos los estados que queremos controlar (salidas de la planta)
C_plant = [ 0, 0, 0, 1, 0, 0;   % Salida 1: u
            0, 0, 0, 0, 0, 1 ]; % Salida 2: r

% --- CORRECCIÓN FUNDAMENTAL ---
% El algoritmo 'mixsyn' puede fallar si la planta tiene ceros en el eje
% imaginario o es numéricamente "impropia". Añadir una matriz D muy pequeña
% pero no nula (haciendo la planta "regular") mejora la condición numérica
% del problema y permite que el algoritmo converja.
D_plant = 1e-6 * eye(size(C_plant, 1), size(B, 2));

% Crear el objeto de espacio de estados para la planta (vehículo)
G = ss(A, B, C_plant, D_plant);

%% 2. Diseño de los Filtros de Ponderación para Mixsyn
% W1: Ponderación de la función de sensibilidad S = (I+GK)^-1
%     Un W1 grande a bajas frecuencias asegura un buen seguimiento.
%     El integrador (1/s) garantiza error nulo en estado estacionario.
s = tf('s');
M = 1.5; % Margen de rendimiento deseado
wb = 0.5;  % Ancho de banda del lazo cerrado (rad/s)
W1 = (s/M + wb) / (s + wb*1e-4); % Integrador aproximado
W1 = [W1, 0; 0, W1]; % Aplicar a ambas salidas

% W2: Ponderación del esfuerzo de control KS = K(I+GK)^-1
%     Limita la magnitud de la señal de control.
W2 = 0.1 * eye(size(B, 2)); % Penalización constante sobre el control

% W3: Ponderación de la sensibilidad complementaria T = GK(I+GK)^-1
%     Asegura robustez y atenuación de ruido de alta frecuencia.
W3 = []; % Se deja vacío, ya que W1 y W2 suelen ser suficientes.

%% 3. Síntesis del Controlador H∞ con Mixsyn
fprintf('Diseñando el controlador de seguimiento con mixsyn...\n');

% mixsyn encuentra un controlador K que minimiza la norma H-infinito de:
% [ W1*S ]
% [ W2*KS]
% [ W3*T ]
[K, CL, gam] = mixsyn(G, W1, W2, W3);

if isempty(K)
    error('No se pudo diseñar el controlador con mixsyn.');
else
    fprintf('¡Controlador de seguimiento diseñado con éxito!\n');
    fprintf('Gamma alcanzado: %.4f\n', gam);
end

disp('Controlador dinámico de seguimiento H-infinito (K):');
disp(K);

%% 4. Simulación de la Respuesta al Escalón (Seguimiento)
fprintf('\nGenerando el gráfico de respuesta al escalón de seguimiento...\n');

% Construir el sistema en lazo cerrado usando la función feedback.
% T = GK(I+GK)^-1 es la función de transferencia de la referencia a la salida.
sys_cl_tracking = feedback(G * K, eye(size(G, 1)));

% Generar el gráfico de respuesta al escalón
figure;
step(sys_cl_tracking, 40);
title('Respuesta de Seguimiento a un Escalón de Referencia (Controlador Mixsyn H∞)');
xlabel('Tiempo (segundos)');
ylabel('Amplitud de las Salidas');
grid on;
legend('Velocidad de Avance (u)', 'Velocidad de Guiñada (r)', 'Location', 'best');

