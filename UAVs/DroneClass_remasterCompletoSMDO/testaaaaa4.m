close all
clearvars
clc

% Parameters for the spring-mass-damper system
m = 1;  % Mass (kg)
k = 10; % Spring constant (N/m)
c = 2;  % Damping coefficient (Ns/m)

% Controller gains
Kp = 100;
Kd = 10;

% Desired position
desired_position = 1;  % Desired position (m)

% Initial conditions
initial_state = [1; 10; 0; 0];            % Initial [position; velocity; disturbance; disturbance rate]
initial_estimated_state = [0; 0; 0; 0];   % Initial estimated state [position; velocity; disturbance; disturbance rate]

% Extended observer design (with dynamic disturbance model)
A = [0, 1, 0, 0; 
    -k/m, -c/m, 1/m, 0;
    0, 0, 0, 1; 
    0, 0, -1, 0];  % Includes disturbance and its rate of change

B = [0; 1/m; 0; 0];
C = [1, 0, 0, 0];  % Measurement matrix for a 4-state system
L = place(A', C', [-7, -8, -15, -22])';  % Gain for extended observer

% Extended simulation settings
dt = 0.01;
t = 0:dt:30;
num_steps = length(t);

% Generate multiple differentiable disturbances at different intervals
disturbance = zeros(1, num_steps);

% Disturbance 1: Smooth sine wave from t = 3 to t = 5
disturbance_time_1 = (t >= 3) & (t <= 5);
disturbance(disturbance_time_1) = 5 * sin(2 * pi * (t(disturbance_time_1) - 3) / 2);

% Disturbance 2: Gaussian centered at t = 10 with a width of 1 second
disturbance = disturbance + 3 * exp(-((t - 10).^2) / (2 * 0.5^2));

% Disturbance 3: Sine wave from t = 15 to t = 18
disturbance_time_3 = (t >= 15) & (t <= 18);
disturbance(disturbance_time_3) = disturbance(disturbance_time_3) + 4 * sin(pi * (t(disturbance_time_3) - 15));

% Disturbance 4: Quadratic burst from t = 20 to t = 22
disturbance_time_4 = (t >= 20) & (t <= 22);
disturbance(disturbance_time_4) = disturbance(disturbance_time_4) + 12 * (t(disturbance_time_4) - 21).^2;

% Disturbance 5: Combination of two sine waves from t = 25 to t = 28
disturbance_time_5 = (t >= 25) & (t <= 28);
disturbance(disturbance_time_5) = disturbance(disturbance_time_5) + 3 * sin(2 * pi * (t(disturbance_time_5) - 25)) + 2 * sin(4 * pi * (t(disturbance_time_5) - 25));

% State storage
x = zeros(4, num_steps);       % Actual state
x_hat = zeros(4, num_steps);    % Estimated state (includes disturbance and its rate)
position_error = zeros(1, num_steps);
control_input = zeros(1, num_steps);

% Initial values
x(:, 1) = initial_state;
x_hat(:, 1) = initial_estimated_state;

% Simulation loop
for i = 1:num_steps-1
    % Position error and PD control to compensate estimated disturbance and rate
    error_hat = desired_position - x_hat(1, i);
    error_dot_hat = -x_hat(2, i);
    
    % Control input based on estimated state (compensates estimated disturbance and rate of change)
    % Weights applied to x_hat(3) and x_hat(4) help tune the disturbance compensation
    u_hat = Kp * error_hat + Kd * error_dot_hat - 0.9 * x_hat(3, i) - 0.3 * x_hat(4, i);  % Compensates for disturbance and disturbance rate estimate
    
    % Real system update with actual disturbance
    dx = A * x(:, i) + B * (u_hat + disturbance(i));
    x(:, i+1) = x(:, i) + dx * dt;
    position_error(i) = desired_position - x(1, i+1);
    
    % Observer state update (position, velocity, disturbance, and disturbance rate)
    y = C * x(:, i);  % Measured position
    dx_hat = A * x_hat(:, i) + B * u_hat + L * (y - C * x_hat(:, i));
    x_hat(:, i+1) = x_hat(:, i) + dx_hat * dt;
    
    % Store control input
    control_input(i) = u_hat;
end

% Plot results
figure;

% Real position vs estimated position
subplot(4,1,1);
hold on;
plot(t, x(1, :), 'r', 'LineWidth', 1.5);
plot(t, x_hat(1, :), 'b--', 'LineWidth', 1.5);
legend('Real Position', 'Estimated Position');
title('Real Position vs. Estimated Position');
xlabel('Time (s)');
ylabel('Position (m)');

% Control input
subplot(4,1,2);
plot(t, control_input, 'k', 'LineWidth', 1.5);
title('Control Input');
xlabel('Time (s)');
ylabel('Control Effort (N)');

% Real disturbance vs estimated disturbance
subplot(4,1,3);
hold on;
plot(t, disturbance, 'r', 'LineWidth', 1.5);
plot(t, x_hat(3, :), 'b--', 'LineWidth', 1.5);
legend('Real Disturbance', 'Estimated Disturbance');
title('Real Disturbance vs. Estimated Disturbance');
xlabel('Time (s)');
ylabel('Disturbance Force (N)');

% Disturbance rate estimate
subplot(4,1,4);
hold on;
plot(t, x_hat(4, :), 'm-', 'LineWidth', 1.5);
title('Estimated Rate of Disturbance');
xlabel('Time (s)');
ylabel('Rate of Disturbance (N/s)');

% Simulation complete
disp('Simulation complete. Check plots for disturbance compensation.');
