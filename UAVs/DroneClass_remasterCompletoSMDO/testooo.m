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
error_no_observer = zeros(1, num_steps);
error_with_observer = zeros(1, num_steps);
control_input = zeros(1, num_steps);
control_input_hat = zeros(1, num_steps);

% Initial values
x(:, 1) = initial_state;
x_hat(:, 1) = initial_estimated_state;

% Simulation loop
for i = 1:num_steps-1
    % Error calculation for PD control
    error = desired_position - x(1, i);
    error_dot = -x(2, i);  % Derivative of error (velocity error)
    
    % PD control
    u = Kp * error + Kd * error_dot;
    
    % Without observer
    dx = A * x(:, i) + B * (u + disturbance(i));
    x(:, i+1) = x(:, i) + dx * dt;
    error_no_observer(i) = desired_position - x(1, i+1);
    
    % With observer
    y = C * x(:, i);  % Measurement
    error_hat = desired_position - x_hat(1, i);
    error_dot_hat = -x_hat(2, i);
    
    % Control based on estimated state
    u_hat = Kp * error_hat + Kd * error_dot_hat;
    
    % Observer state update
    dx_hat = A * x_hat(:, i) + B * u_hat + L * (y - C * x_hat(:, i));
    x_hat(:, i+1) = x_hat(:, i) + dx_hat * dt;
    
    % Store errors
    error_with_observer(i) = desired_position - x_hat(1, i+1);
    control_input(i) = u;
    control_input_hat(i) = u_hat;
end

% Plotting
figure;

% Position error without observer
subplot(3,2,1);
hold on;
plot(t, error_no_observer, 'r');
plot(t, error_with_observer, 'b');
title('Position Error');
legend('No observer error', 'L observer error');
xlabel('Time (s)');
ylabel('Error (m)');

% Actual position vs. Estimated position
subplot(3,2,2);
hold on;
plot(t, x(1, :), 'r');
plot(t, x_hat(1, :), 'b--');
legend('Actual Position', 'Estimated Position');
title('Actual vs. Estimated Position');
xlabel('Time (s)');
ylabel('Position (m)');

% Actual velocity vs. Estimated velocity
subplot(3,2,3);
hold on;
plot(t, x(2, :), 'r');
plot(t, x_hat(2, :), 'b--');
legend('Actual Velocity', 'Estimated Velocity');
title('Actual vs. Estimated Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

% Control input
subplot(3,2,4);
hold on;
plot(t, control_input, 'k');
plot(t, control_input_hat, 'g');
title('Control Input');
legend('no obs', 'L obs');
xlabel('Time (s)');
ylabel('Control Effort (N)');

% Convergence of position and velocity estimates
subplot(3,2,5);
plot(t, abs(x(1,:) - x_hat(1,:)), 'g');
hold on;
plot(t, abs(x(2,:) - x_hat(2,:)), 'm');
legend('Position Estimation Error', 'Velocity Estimation Error');
title('Estimation Error (Observer)');
xlabel('Time (s)');
ylabel('Error');

% Disturbance plot
subplot(3,2,6);
plot(t, disturbance, 'r');
title('Disturbance Force Applied');
xlabel('Time (s)');
ylabel('Force (N)');

% Display result
disp('Simulation complete. Review the plots for comparison.');
