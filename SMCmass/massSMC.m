% Ines Alejandro Garcia Mosqueda
% Sliding Mode Control for a mass-spring-damper system 

% System parameters
m = 1;          % Mass [kg]
c = 0.5;        % Damping coefficient [N*s/m]
k = 2;          % Spring constant [N/m]
A = [0 1; -k/m -c/m];  % State matrix
B = [0; 1/m];          % Input matrix

% Desired position
x_d = 2;        % Desired position [m]

% Sliding surface parameter
lambda = 10;     % Sliding surface parameter

% Simulation parameters
dt = 0.01;         % Time step
T = 10;            % Total simulation time
time = 0:dt:T;     % Time vector

% Control parameters
K = 10;            % Sliding mode control gain
epsilon = 0.1;     % Boundary layer for continuous approximation (chattering reduction) Not the best way XD

% Initial conditions
x = [0; 0];        % Initial state [position; velocity]

x_history = zeros(2, length(time));
u_history = zeros(1, length(time));

for k = 1:length(time)
    e = x(1) - x_d;
    sigma = x(2) + lambda * e;
    u_eq = -(lambda * x(2) + k/m * (x(1) - x_d) + c/m * x(2));  % Equivalent control
    u_sw = -K * sat(sigma, epsilon); % Switching control with boundary layer
    u = u_eq + u_sw;
    dx = A * x + B * u;
    x = x + dx * dt;
    x_history(:, k) = x;
    u_history(k) = u;
end

figure;
subplot(3, 1, 1);
plot(time, x_history(1, :), 'r', 'LineWidth', 2); hold on;
plot(time, x_history(2, :), 'b--', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('States');
legend('Position (x_1)', 'Velocity (x_2)');
title('System States under SMC for Position Tracking');

subplot(3, 1, 2);
plot(time, u_history, 'k', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Control Input (u)');
title('Control Input (u)');

subplot(3, 1, 3);
plot(x_history(1, :), x_history(2, :), 'g', 'LineWidth', 2);
xlabel('Position (x_1)');
ylabel('Velocity (x_2)');
title('Phase Diagram (Position vs. Velocity)');
grid on;

function u = sat(sigma, epsilon)
    u = sigma / epsilon * (abs(sigma) <= epsilon) + sign(sigma) * (abs(sigma) > epsilon);
end
