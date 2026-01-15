% --- Código de MATLAB para replicar el gráfico ---

% 1. Definir los datos (valores estimados de la imagen)
% Cada fila es un país, cada columna es una serie de datos
% Columnas: [Small Satellites, Development place, UNOOSA Register]
data = [ ...
    6, 68, 15;  % Argentina
    0,  1,  1;  % Bolivia
    4, 34, 44;  % Brasil
    4,  7,  8;  % Chile
    3,  3,  3;  % Colombia
    2,  2,  2;  % Ecuador
    0,  1,  1;  % Paraguay
    4,  5,  4;  % Perú
    0,  1, 49;  % Uruguay
    0,  3,  3   % Venezuela
];

% 2. Definir las etiquetas para el eje Y (los países)
% El orden es de abajo hacia arriba, como los dibuja MATLAB
categories = {'Argentina', 'Bolivia', 'Brasil', 'Chile', ...
              'Colombia', 'Ecuador', 'Paraguay', 'Perú', ...
              'Uruguay', 'Venezuela'};

% 3. Create the horizontal bar chart
figure; % Creates a new figure window
b = barh(data, 'grouped');

% 4. Adjust labels and title
% Assigns the country labels to the Y-axis
set(gca, 'YTickLabel', categories);

% Title and X-axis label
title('SATELLITES LAUNCHED IN SOUTH AMERICA');
xlabel('Number of Satellites');

% 5. Add the legend
legend('Small Satellites', 'Development place', 'UNOOSA Register', ...
       'Location', 'east'); % Moves the legend to the east (right)

% 6. Optional: Invert the Y-axis so Venezuela is at the top
% (Just like in your original image)
set(gca, 'YDir', 'reverse');

% 7. Optional: Add the grid
grid on;

% --- End of code ---