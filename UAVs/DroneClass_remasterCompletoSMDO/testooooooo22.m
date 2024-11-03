close all
clearvars
clc

% Parameters for the spring-mass-damper system
m = 1;  % Mass (kg)
k = 10; % Spring constant (N/m)
c = 2;  % Damping coefficient (Ns/m)

% PD controller gains
Kp = 100;
Kd = 10;

% Desired position
desired_position = 1;  % Desired position (m)

% Initial conditions
initial_state = [1; 10];  % Initial [position; velocity]
initial_estimated_state = [0; 0];  % Initial estimated state

% Observer design (Luenberger Observer)
A = [0, 1; -k/m, -c/m];
B = [0; 1/m];
C = [1, 0];
L = place(A', C', [-2, -3])';  % Observer gain for stability

% Simulation settings
dt = 0.01;
t = 0:dt:10;
num_steps = length(t);

% Differentiable disturbance (sine wave) applied from t = 3 to t = 5 seconds
disturbance = zeros(1, num_steps);
disturbance_time_indices = (t >= 3) & (t <= 5);
disturbance(disturbance_time_indices) = 5 * sin(2 * pi * (t(disturbance_time_indices) - 3) / 2);

% State storage
x = zeros(2, num_steps);       % Actual state
x_hat = zeros(2, num_steps);    % Estimated state
position_error = zeros(1, num_steps);
control_input = zeros(1, num_steps);

% Initial values
x(:, 1) = initial_state;
x_hat(:, 1) = initial_estimated_state;

% Simulation loop
for i = 1:num_steps-1
    % Error calculation for PD control based on estimated state
    error_hat = desired_position - x_hat(1, i);
    error_dot_hat = -x_hat(2, i);  % Derivative of error (velocity error)
    
    % PD control based on estimated state
    u_hat = Kp * error_hat + Kd * error_dot_hat;
    
    % Update actual system with disturbance
    dx = A * x(:, i) + B * (u_hat + disturbance(i));
    x(:, i+1) = x(:, i) + dx * dt;
    position_error(i) = desired_position - x(1, i+1);
    
    % Observer state update based on measurement
    y = C * x(:, i);  % Measurement
    dx_hat = A * x_hat(:, i) + B * u_hat + L * (y - C * x_hat(:, i));
    x_hat(:, i+1) = x_hat(:, i) + dx_hat * dt;
    
    % Store control input
    control_input(i) = u_hat;
end

% Plotting
figure;

% Actual position vs. Estimated position
subplot(3,1,1);
hold on;
plot(t, x(1, :), 'r', 'LineWidth', 1.5);
plot(t, x_hat(1, :), 'b--', 'LineWidth', 1.5);
legend('Actual Position', 'Estimated Position');
title('Actual vs. Estimated Position');
xlabel('Time (s)');
ylabel('Position (m)');

% Control input
subplot(3,1,2);
plot(t, control_input, 'k', 'LineWidth', 1.5);
title('Control Input');
xlabel('Time (s)');
ylabel('Control Effort (N)');

% Disturbance plot
subplot(3,1,3);
plot(t, disturbance, 'r', 'LineWidth', 1.5);
title('Disturbance Force Applied');
xlabel('Time (s)');
ylabel('Force (N)');

% Display result
disp('Simulation complete. Review the plots for observer-based control.');
