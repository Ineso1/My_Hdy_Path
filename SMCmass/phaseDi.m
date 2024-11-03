close all
clearvars
clc

% Parameters
m = 1;          % Mass (kg)
c = 0.5;        % Damping coefficient
k = 2;          % Spring constant (N/m)

% Define the system of equations
dxdt = @(t, y) [y(2); -c/m*y(2) - k/m*y(1)];

% Phase diagram
tspan = [0, 20];            % Time range
initial_conditions = [-2, 0; 2, 0; 0, 2; 0, -2]; % Initial conditions for trajectories
colors = {'g', 'b', 'm', 'r'}; % Colors: green, magenta, pink, red

figure;
hold on;

for i = 1:size(initial_conditions, 1)
    [~, Y] = ode45(dxdt, tspan, initial_conditions(i, :));
    plot(Y(:,1), Y(:,2), 'Color', colors{i}, 'LineWidth', 1.5); % Position vs. Velocity
end

xlabel('Position (x)');
ylabel('Velocity (v)');
title('Phase Diagram of Mass-Spring-Damper System');
grid on;

% Vector field for the phase diagram
[x, v] = meshgrid(-3:0.3:3, -3:0.3:3); % Grid for position and velocity
dx = v; % dx/dt = velocity
dv = -c/m * v - k/m * x; % dv/dt = acceleration

hold on
quiver(x, v, dx, dv, 'k');
xlabel('x_1');
ylabel('x_2');
title('Vector Field System');
axis tight;
grid on;
