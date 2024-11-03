close all
clearvars
clc

% Parámetros del sistema masa-resorte-amortiguador
m = 1;  % Masa (kg)
k = 10; % Constante del resorte (N/m)
c = 2;  % Coeficiente de amortiguamiento (Ns/m)

% Ganancias del controlador PID
Kp = 100;
Kd = 10;
Ki = 5;  % Ganancia del término integral para compensar la perturbación

% Posición deseada
desired_position = 1;  % Posición deseada (m)

% Condiciones iniciales
initial_state = [1; 10];       % Estado inicial [posición; velocidad]
initial_estimated_state = [0; 0; 0];  % Estado estimado inicial [posición; velocidad; perturbación]

% Diseño del observador extendido (con estimación de perturbación)
A = [0, 1, 0; -k/m, -c/m, 1/m; 0, 0, 0];  % Incluye perturbación como estado adicional
B = [0; 1/m; 0];
C = [1, 0, 0];
L = place(A', C', [-7, -10, -12])';  % Ganancia del observador

% Configuración de la simulación extendida a 30 segundos
dt = 0.01;
t = 0:dt:30;
num_steps = length(t);

% Generación de múltiples perturbaciones diferenciables en diferentes intervalos
disturbance = zeros(1, num_steps);

% Perturbación 1: Senoidal suave entre t = 3 y t = 5
disturbance_time_1 = (t >= 3) & (t <= 5);
disturbance(disturbance_time_1) = 5 * sin(2 * pi * (t(disturbance_time_1) - 3) / 2);

% Perturbación 2: Gaussiana centrada en t = 10 con ancho de 1 segundo
disturbance = disturbance + 3 * exp(-((t - 10).^2) / (2 * 0.5^2));

% Perturbación 3: Senoidal entre t = 15 y t = 18
disturbance_time_3 = (t >= 15) & (t <= 18);
disturbance(disturbance_time_3) = disturbance(disturbance_time_3) + 4 * sin(pi * (t(disturbance_time_3) - 15));

% Perturbación 4: Ráfaga cuadrática entre t = 20 y t = 22
disturbance_time_4 = (t >= 20) & (t <= 22);
disturbance(disturbance_time_4) = disturbance(disturbance_time_4) + 2 * (t(disturbance_time_4) - 21).^2;

% Perturbación 5: Combinación de dos ondas seno entre t = 25 y t = 28
disturbance_time_5 = (t >= 25) & (t <= 28);
disturbance(disturbance_time_5) = disturbance(disturbance_time_5) + 3 * sin(2 * pi * (t(disturbance_time_5) - 25)) + 2 * sin(4 * pi * (t(disturbance_time_5) - 25));

% Almacenamiento de estados
x = zeros(2, num_steps);       % Estado real
x_hat = zeros(3, num_steps);    % Estado estimado (incluye perturbación)
position_error = zeros(1, num_steps);
control_input = zeros(1, num_steps);

% Valores iniciales
x(:, 1) = initial_state;
x_hat(:, 1) = initial_estimated_state;

% Bucle de simulación
for i = 1:num_steps-1
    % Error de posición y control integral para compensación de perturbación
    error_hat = desired_position - x_hat(1, i);
    error_dot_hat = -x_hat(2, i);
    
    % Controlador PID ajustado para compensar la perturbación estimada
    u_hat = Kp * error_hat + Kd * error_dot_hat - x_hat(3, i);  % Compensa perturbación estimada
    
    % Actualización del sistema real con perturbación
    dx = A(1:2, 1:2) * x(:, i) + B(1:2) * (u_hat + disturbance(i));
    x(:, i+1) = x(:, i) + dx * dt;
    position_error(i) = desired_position - x(1, i+1);
    
    % Actualización del observador con estado de perturbación
    y = C(1:2) * x(:, i);  % Medición de posición
    dx_hat = A * x_hat(:, i) + B * u_hat + L * (y - C * x_hat(:, i));
    x_hat(:, i+1) = x_hat(:, i) + dx_hat * dt;
    
    % Almacenamiento de la señal de control
    control_input(i) = u_hat;
end

% Gráficas
figure;

% Posición real vs estimada
subplot(3,1,1);
hold on;
plot(t, x(1, :), 'r', 'LineWidth', 1.5);
plot(t, x_hat(1, :), 'b--', 'LineWidth', 1.5);
legend('Posición Real', 'Posición Estimada');
title('Posición Real vs. Estimada');
xlabel('Tiempo (s)');
ylabel('Posición (m)');

% Entrada de control
subplot(3,1,2);
plot(t, control_input, 'k', 'LineWidth', 1.5);
title('Entrada de Control');
xlabel('Tiempo (s)');
ylabel('Esfuerzo de Control (N)');

% Perturbación real vs estimada
subplot(3,1,3);
hold on;
plot(t, disturbance, 'r', 'LineWidth', 1.5);
plot(t, x_hat(3, :), 'b--', 'LineWidth', 1.5);
legend('Perturbación Real', 'Perturbación Estimada');
title('Perturbación Real vs. Estimada');
xlabel('Tiempo (s)');
ylabel('Fuerza (N)');












% Parámetros del sistema masa-resorte-amortiguador
m = 1;  % Masa (kg)
k = 10; % Constante del resorte (N/m)
c = 2;  % Coeficiente de amortiguamiento (Ns/m)

% Ganancias del controlador PID
Kp = 100;
Kd = 10;
Ki = 5;  % Ganancia del término integral para compensar la perturbación

% Posición deseada
desired_position = 1;  % Posición deseada (m)

% Condiciones iniciales
initial_state = [1; 10];       % Estado inicial [posición; velocidad]
initial_estimated_state = [0; 0; 0];  % Estado estimado inicial [posición; velocidad; perturbación]

% Diseño del observador extendido (con estimación de perturbación)
A = [0, 1, 0; -k/m, -c/m, 1/m; 0, 0, 0];  % Incluye perturbación como estado adicional
B = [0; 1/m; 0];
C = [1, 0, 0];
L = place(A', C', [-7, -10, -12])';  % Ganancia del observador

% Configuración de la simulación extendida a 30 segundos
dt = 0.01;
t = 0:dt:30;
num_steps = length(t);

% Generación de múltiples perturbaciones diferenciables en diferentes intervalos
disturbance = zeros(1, num_steps);

% Perturbación 1: Senoidal suave entre t = 3 y t = 5
disturbance_time_1 = (t >= 3) & (t <= 5);
disturbance(disturbance_time_1) = 5 * sin(2 * pi * (t(disturbance_time_1) - 3) / 2);

% Perturbación 2: Gaussiana centrada en t = 10 con ancho de 1 segundo
disturbance = disturbance + 3 * exp(-((t - 10).^2) / (2 * 0.5^2));

% Perturbación 3: Senoidal entre t = 15 y t = 18
disturbance_time_3 = (t >= 15) & (t <= 18);
disturbance(disturbance_time_3) = disturbance(disturbance_time_3) + 4 * sin(pi * (t(disturbance_time_3) - 15));

% Perturbación 4: Ráfaga cuadrática entre t = 20 y t = 22
disturbance_time_4 = (t >= 20) & (t <= 22);
disturbance(disturbance_time_4) = disturbance(disturbance_time_4) + 2 * (t(disturbance_time_4) - 21).^2;

% Perturbación 5: Combinación de dos ondas seno entre t = 25 y t = 28
disturbance_time_5 = (t >= 25) & (t <= 28);
disturbance(disturbance_time_5) = disturbance(disturbance_time_5) + 3 * sin(2 * pi * (t(disturbance_time_5) - 25)) + 2 * sin(4 * pi * (t(disturbance_time_5) - 25));

% Almacenamiento de estados
x = zeros(2, num_steps);       % Estado real
x_hat = zeros(3, num_steps);    % Estado estimado (incluye perturbación)
position_error = zeros(1, num_steps);
control_input = zeros(1, num_steps);

% Valores iniciales
x(:, 1) = initial_state;
x_hat(:, 1) = initial_estimated_state;

% Bucle de simulación
for i = 1:num_steps-1
    % Error de posición y control integral para compensación de perturbación
    error_hat = desired_position - x_hat(1, i);
    error_dot_hat = -x_hat(2, i);
    
    % Controlador PID ajustado para compensar la perturbación estimada
    u_hat = Kp * error_hat + Kd * error_dot_hat;  % Compensa perturbación estimada
    
    % Actualización del sistema real con perturbación
    dx = A(1:2, 1:2) * x(:, i) + B(1:2) * (u_hat + disturbance(i));
    x(:, i+1) = x(:, i) + dx * dt;
    position_error(i) = desired_position - x(1, i+1);
    
    % Actualización del observador con estado de perturbación
    y = C(1:2) * x(:, i);  % Medición de posición
    dx_hat = A * x_hat(:, i) + B * u_hat + L * (y - C * x_hat(:, i));
    x_hat(:, i+1) = x_hat(:, i) + dx_hat * dt;
    
    % Almacenamiento de la señal de control
    control_input(i) = u_hat;
end

% Gráficas
figure;
% Posición real vs estimada
subplot(3,1,1);
hold on;
plot(t, x(1, :), 'r', 'LineWidth', 1.5);
plot(t, x_hat(1, :), 'b--', 'LineWidth', 1.5);
legend('Posición Real', 'Posición Estimada');
title('Posición Real vs. Estimada');
xlabel('Tiempo (s)');
ylabel('Posición (m)');

% Entrada de control
subplot(3,1,2);
plot(t, control_input, 'k', 'LineWidth', 1.5);
title('Entrada de Control');
xlabel('Tiempo (s)');
ylabel('Esfuerzo de Control (N)');

% Perturbación real vs estimada
subplot(3,1,3);
hold on;
plot(t, disturbance, 'r', 'LineWidth', 1.5);
plot(t, x_hat(3, :), 'b--', 'LineWidth', 1.5);
legend('Perturbación Real', 'Perturbación Estimada');
title('Perturbación Real vs. Estimada');
xlabel('Tiempo (s)');
ylabel('Fuerza (N)');

% Resultado de la simulación
disp('Simulación completa. Revisa las gráficas para la compensación de perturbación.');
