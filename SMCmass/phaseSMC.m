% Parameters
m = 1;          % Mass (kg)
c = 0.5;        % Damping coefficient
k = 2;          % Spring constant (N/m)

% SMC Parameters
x_d = 0;        % Desired position
lambda = 1;    % Sliding surface parameter
K = 10;         % Control gain

% Define the controlled system of equations with SMC
dxdt_smc = @(t, y) [
    y(2);
    -c/m*y(2) - k/m*y(1) + smc_control(y, x_d, m, c, k, lambda, K)
];

% Phase diagram with SMC
tspan = [0, 20];                      % Time range
initial_conditions = [-2, 0; 2, 0; 0, 2; 0, -2]; % Initial conditions for trajectories
colors = {'g', 'b', 'm', 'r'};        % Colors: green, blue, magenta, red

figure;
hold on;

for i = 1:size(initial_conditions, 1)
    [~, Y] = ode45(dxdt_smc, tspan, initial_conditions(i, :));
    plot(Y(:,1), Y(:,2), 'Color', colors{i}, 'LineWidth', 1.5); % Position vs. Velocity
end

xlabel('Position (x)');
ylabel('Velocity (v)');
title('Phase Diagram of Mass-Spring-Damper System with SMC');
grid on;

% Vector field with SMC
[x, v] = meshgrid(-5:0.7:5, -5:0.7:5); % Reduced mesh density
dx = v; % dx/dt = velocity
dv = zeros(size(x)); % dv/dt will be updated with control

for i = 1:numel(x)
    dv(i) = -c/m * v(i) - k/m * x(i) + smc_control([x(i); v(i)], x_d, m, c, k, lambda, K);
end

hold on
quiver(x, v, dx, dv, 'k', 'AutoScale', 'on', 'AutoScaleFactor', 0.5); % Smaller vector size
xlabel('x_1');
ylabel('x_2');
title('Vector Field of Controlled System with SMC');
axis equal;
axis([-5 5 -5 5]); % Cropped range
grid on;

% Sliding surface plot
x_range = -5:0.1:5;
v_surface = -lambda * (x_range - x_d); % Calculate v for sigma = 0
plot(x_range, v_surface, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Sliding Surface (sigma = 0)');

legend('condition 1', 'condition 2', 'condition 3', 'condition 4', 'field', 'Sliding Surface (sigma = 0)');

% SMC Control function
function u = smc_control(y, x_d, m, c, k, lambda, K)
    e = y(1) - x_d;
    sigma = y(2) + lambda * e;
    u_eq = -(lambda * y(2) + k/m * (y(1) - x_d) + c/m * y(2)); % Equivalent control
    u_sw = -K * sign(sigma); % Switching control
    u = u_eq + u_sw;
end
