%% Ines Alejandro Garcia Mosqueda
% Pendulum LQR control with and without observer comp
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Clear everything
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clearvars
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Car and pendulum params
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m_p = 0.2;       % Mass of the pendulum (kg)
m_c = 1.0;       % Mass of the cart (kg)
l = 0.5;         % Length of the pendulum (m)
g = 9.81;        % Acceleration due to gravity (m/s^2)

A = [0 1 0 0;
     0 0 -(m_p * g) / (m_c + m_p) 0;
     0 0 0 1;
     0 0 (g * (m_c + m_p)) / (l * (m_c + m_p)) 0];

B = [0;
     1 / (m_c + m_p);
     0;
     -1 / (l * (m_c + m_p))];

C = [1 0 0 0;  % Position and angle measure
     0 0 1 0];

D = [0; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Observer and LQR params
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Observer
desired_eigenvalues = [-10, -11, -12, -13];  
L = place(A', C', desired_eigenvalues)';  %Observer matrix

% LQR vars
Q = diag([10 1 10 1]);  % State cost matrix
R = 1;                  % Control input cost
K = lqr(A, B, Q, R);    % State feedback gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Simulation params
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T = 10;                
dt = 0.01;            
time = 0:dt:T;         

% Initial conditions
x0 = [0; 0; 0.5; 0];   % [x; x_dot; theta; theta_dot]
x_hat0 = [0; 0; 0; 0]; % Initial estimated state for observer
x_no_observer = x0;     % Without the observer state

% some arrs
x = zeros(4, length(time));      
x_hat = zeros(4, length(time));  
x_no_obs = zeros(4, length(time));  
u = zeros(1, length(time));    
u_no_obs = zeros(1, length(time));  

% arrays for error 
error_with_observer = zeros(4, length(time));
error_without_observer = zeros(4, length(time));  

% Initial conditions
x(:,1) = x0;
x_hat(:,1) = x_hat0;
x_no_obs(:,1) = x_no_observer;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Disturbance addition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disturbance = zeros(1, length(time));
disturbance(time >= 3 & time < 3.5) = 10;  


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Animation loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Desired state
x_ref = [0; 0; 0; 0];  % [x, x_dot, theta, theta_dot]

for k = 1:length(time)-1

    % Control input with and without observer
    u(k) = -K * x_hat(:,k);          
    u_no_obs(k) = -K * x_no_obs(:,k); 
    
    % True system dynamics with and without observer
    dx = A * x(:,k) + B * (u(k) + disturbance(k));
    x(:,k+1) = x(:,k) + dx * dt;
    
    dx_no_obs = A * x_no_obs(:,k) + B * (u_no_obs(k) + disturbance(k));
    x_no_obs(:,k+1) = x_no_obs(:,k) + dx_no_obs * dt;
    
    % Output measure for the observer
    y = C * x(:,k);
    
    % Estimate the state with the observer
    dx_hat = A * x_hat(:,k) + B * u(k) + L * (y - C * x_hat(:,k));  % Observer correction
    x_hat(:,k+1) = x_hat(:,k) + dx_hat * dt;
    
    % Calculate estimation error (true state - estimated state)
    error_with_observer(:,k+1) = x(:,k+1) - x_hat(:,k+1); 
    
    % Calculate the state error without observer (true state - reference state)
    error_without_observer(:,k+1) = x_no_obs(:,k+1) - x_ref;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Plt
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
subplot(3,1,1);
plot(time, x(1,:), 'b', time, x_hat(1,:), 'r--');
xlabel('Time (s)');
ylabel('Cart Position (m)');
legend('True Position', 'Estimated Position');
title('Cart Position vs Estimated Position (With Observer)');

subplot(3,1,2);
plot(time, x(3,:), 'b', time, x_hat(3,:), 'r--');
xlabel('Time (s)');
ylabel('Pendulum Angle (rad)');
legend('True Angle', 'Estimated Angle');
title('Pendulum Angle vs Estimated Angle (With Observer)');

subplot(3,1,3);
plot(time, u);
xlabel('Time (s)');
ylabel('Control Input (N)');
title('Control Input (With Observer)');

figure;
subplot(3,1,1);
plot(time, x(1,:), 'b', time, x_no_obs(1,:), 'g--');
xlabel('Time (s)');
ylabel('Cart Position (m)');
legend('With Observer', 'Without Observer');
title('Cart Position Comparison');

subplot(3,1,2);
plot(time, x(3,:), 'b', time, x_no_obs(3,:), 'g--');
xlabel('Time (s)');
ylabel('Pendulum Angle (rad)');
legend('With Observer', 'Without Observer');
title('Pendulum Angle Comparison');

subplot(3,1,3);
plot(time, disturbance, 'r');
xlabel('Time (s)');
ylabel('Disturbance (N)');
title('Applied Disturbance');

figure;
subplot(2,1,1);
plot(time, error_with_observer(1,:), 'r');
xlabel('Time (s)');
ylabel('Error (m)');
title('Estimation Error in Cart Position (With Observer)');

subplot(2,1,2);
plot(time, error_with_observer(3,:), 'r');
xlabel('Time (s)');
ylabel('Error (rad)');
title('Estimation Error in Pendulum Angle (With Observer)');

figure;
subplot(2,1,1);
plot(time, error_without_observer(1,:), 'g');
xlabel('Time (s)');
ylabel('Error (m)');
title('State Error in Cart Position (Without Observer)');

subplot(2,1,2);
plot(time, error_without_observer(3,:), 'g');
xlabel('Time (s)');
ylabel('Error (rad)');
title('State Error in Pendulum Angle (Without Observer)');
