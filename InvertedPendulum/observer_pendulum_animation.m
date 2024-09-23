%% Ines Alejandro Garcia Mosqueda
% Pendulum LQR control with observer animation
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

% position and angle measure
C = [1 0 0 0; 
     0 0 1 0];

D = [0; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Observer and LQR params
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Observer
desired_eigenvalues = [-10, -11, -12, -13];  
L = place(A', C', desired_eigenvalues)';

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
% x0 = [x; x_dot; theta; theta_dot]
x0 = [0; 0; 0.5; 0];   
x_hat0 = [0; 0; 0; 0]; 

% Init things for this XD
x = zeros(4, length(time));      % True state
x_hat = zeros(4, length(time));  % Estimated state (observer)
u = zeros(1, length(time));      % Control input

% initial conditions
x(:,1) = x0;
x_hat(:,1) = x_hat0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Animation loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% desired state
x_ref = [0; 0; 0; 0];

% Animation figures params
figure;
cart_width = 0.4;    
cart_height = 0.2;   
pendulum_length = l; 
axis([-2 2 -1.5 1.5]); 
hold on;

cart = rectangle('Position', [-cart_width/2, 0, cart_width, cart_height], 'Curvature', 0.1, 'FaceColor', 'blue');
pendulum = line([0, 0], [cart_height/2, cart_height/2 - pendulum_length], 'LineWidth', 2, 'Color', 'red');
ground = line([-2, 2], [0, 0], 'Color', 'black');

for k = 1:length(time)-1
    u(k) = -K * x_hat(:,k);  
    dx = A * x(:,k) + B * u(k);
    x(:,k+1) = x(:,k) + dx * dt;
    y = C * x(:,k);
    dx_hat = A * x_hat(:,k) + B * u(k) + L * (y - C * x_hat(:,k));
    x_hat(:,k+1) = x_hat(:,k) + dx_hat * dt;
    
    % Animation update
    cart_pos = x(1,k);   
    pendulum_angle = x(3,k);   
    cart.Position = [cart_pos - cart_width/2, 0, cart_width, cart_height];
    pendulum_x = cart_pos + pendulum_length * sin(pendulum_angle);
    pendulum_y = cart_height/2 - pendulum_length * cos(pendulum_angle);
    set(pendulum, 'XData', [cart_pos, pendulum_x]);
    set(pendulum, 'YData', [cart_height/2, pendulum_y]);
    pause(0.01);
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
title('Cart Position vs Estimated Position');

subplot(3,1,2);
plot(time, x(3,:), 'b', time, x_hat(3,:), 'r--');
xlabel('Time (s)');
ylabel('Pendulum Angle (rad)');
legend('True Angle', 'Estimated Angle');
title('Pendulum Angle vs Estimated Angle');

subplot(3,1,3);
plot(time, u);
xlabel('Time (s)');
ylabel('Control Input (N)');
title('Control Input');
