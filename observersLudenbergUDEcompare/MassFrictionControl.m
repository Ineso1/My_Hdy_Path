close all
clearvars
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Parameters %%%%

m = 1;                  % Mass
b = 0.5;                % Damping/friction coefficient
Kp = 1;    
Kd = 0.5;     
disturbance = 10;        % External disturbance

Kp = 0;    
Kd = 0;     

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Sim %%%%

dt = 0.01;  
t = 0:dt:25;
n = length(t);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% State-space Model %%%%

% State-space matrices (A, B, C, D)
A = [0 1; 0 -b/m];      % System dynamics matrix
B = [0; 1/m];           % Control input matrix
C = [1 0];              % Output matrix
D = 0;                  % Direct feedthrough

B_pinv = pinv(B);       % B pseudo inverse (2x1 in this case)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% State variables %%%%

x = zeros(2,n);         % x(1) = position, x(2) = velocity
x_hat_L = zeros(2,n);   % State estimate (L observer)
x_hat_UDE = zeros(2,n); % State estimate (UDE observer)
x_hat_SMC = zeros(2,n); % State estimate (Sliding Mode observer)
x_hat_ST = zeros(2, n);        % State estimate (Super-Twisting observer)
ref = ones(1,n);        % Reference signal

dx = zeros(2,n);         % x(1) = position, x(2) = velocity
dx_hat_L = zeros(2,n);   % State estimate (L observer)
dx_hat_UDE = zeros(2,n); % State estimate (UDE observer)
dx_hat_SMC = zeros(2,n); % State estimate (Sliding Mode observer)
dx_hat_ST = zeros(2, n);        % State estimate (Super-Twisting observer)
dref = ones(1,n);        % Reference signal

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Observer gains %%%%

desired_eigenvalues = [-10, -11];  
L = place(A', C', desired_eigenvalues)';
Omega = 15 ;
gamma = 0.1;    % Sliding mode observer gain 
k_d = 10.5;    % Damping term for sliding mode observer

lambda = 0.001;                % Gain for sliding surface
disturbance_bound = 15;      % Estimated bound of disturbance
lambda_0 = 2.1 * sqrt(disturbance_bound);  % Primary super-twisting gain
lambda_1 = 1.1 * disturbance_bound;        % Auxiliary gain

s_history = zeros(1, n);     % Store sliding variable history
eta = 0;                     % Auxiliary variable for super-twisting

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Initial conditions %%%%

x(:,1) = [0; -10];
x_hat_L(:,1) = [0; 0]; 
x_hat_UDE(:,1) = [0; 0]; 
x_hat_SMC(:,1) = [0; 0];
x_hat_ST(:,1) = [0; 0];

dx(:,1) = [0; -10];
dx_hat_L(:,1) = [0; 0]; 
dx_hat_UDE(:,1) = [0; 0]; 
dx_hat_SMC(:,1) = [0; 0];
dx_hat_ST(:,1) = [0; 0];


xi = -Omega * (B_pinv * x(:,1));  % Initial state of UDE observer xi_0

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Control loop %%%%

u = zeros(1,n);             % Control input
u_UDE = zeros(1,n);         % Control input UDE
u_L = zeros(1,n);         % Control input UDE
u_SMC = zeros(1,n);         % Control input UDE

for k = 1:n-1
    % PD control
    e = ref(k) - x(1,k);    % Position error
    de = -x(2,k);           % Velocity error
    u(k) = Kp*e + Kd*de;    % Standard control input
    
    % Apply disturbance some period of time XD
    if k > 5/dt && k < 10/dt
        F_disturbance = disturbance;
    elseif k > 12/dt && k < 15/dt
        F_disturbance = disturbance * 1.5;
    else
        F_disturbance = 0;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Control loop --- State-space equation with disturbance %%%%
    dx(:, k+1) = A * x(:, k) + B * (u(k) + F_disturbance);    % dx = A*x + B*u
    x(:, k+1) = x(:, k) + dt * dx(:, k);                    % Update system states
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Control loop --- Luenberger Observer (L) %%%%
    y = C * x(:, k);    % Measurement 
    dx_hat_L(:, k+1) = A * x_hat_L(:, k) + B * u(k) + L * (y - C * x_hat_L(:, k));  
    x_hat_L(:, k+1) = x_hat_L(:, k) + dt * dx_hat_L(:, k);  % Update Luenberger observer states


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% UDE state update (with disturbance compensation) %%%%
    xi_dot = -Omega * xi - (Omega * Omega * (B_pinv * x(:,k)) + Omega * (B_pinv * A * x(:,k))) - Omega * u(k); 
    xi = xi + dt * xi_dot;  % Update xi
    w_hat = xi + Omega * (B_pinv * x(:,k));  % w_hat(t) = xi + Omega * B^+ x(t)
    dx_hat_UDE(:, k+1) = A * x(:, k) + B * u(k) + B * w_hat;  
    x_hat_UDE(:, k+1) = x_hat_UDE(:, k) + dt * dx_hat_UDE(:, k);  % Update UDE observer states

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% Sliding Mode Observer/Differentiator %%%%
    % Estimate of derivative of position (velocity x_2)
    e_smc = x_hat_SMC(1, k) - x(1, k);    % Position estimation error
    e_smc_vel = x_hat_SMC(2, k) - x(2, k); % Velocity estimation error

    % Sliding mode injection term with both position and velocity errors
    v_injection = -gamma * sign(e_smc) - k_d * e_smc_vel;

    dx_hat_SMC(1, k+1) = x_hat_SMC(2, k);      % Update for position estimate
    dx_hat_SMC(2, k+1) = v_injection;          % Update for velocity estimate with injection

    x_hat_SMC(:, k+1) = x_hat_SMC(:, k) + dt * dx_hat_SMC(:, k);  % Update SMC observer states

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Super-Twisting Observer %%%%
    
    % Desired reference and errors
    x_d = ref(k);
    e_ST = x_d - x(1,k);             % Position error
    de_ST = 0 - x(2,k);              % Velocity error (assuming desired velocity is zero)
    sigma = de_ST + lambda * e_ST;      % Sliding surface
    
    % Sliding variable for the observer
    s = sigma + x_hat_ST(2,k);   % Augmented sliding variable with auxiliary term (z)

    % Super-Twisting control law for v
    v_injection_ST = -lambda_0 * sign(s) * sqrt(abs(s)) - eta;

    % Update eta based on sign of s
    eta = eta + dt * lambda_1 * sign(s);

    % Observer dynamics: update for position and velocity estimates
    dx_hat_ST(1, k+1) = x_hat_ST(2, k);               % Position estimate update
    dx_hat_ST(2, k+1) = v_injection_ST;               % Velocity estimate with super-twisting injection

    % Integrate the observer state estimates
    x_hat_ST(:, k+1) = x_hat_ST(:, k) + dt * dx_hat_ST(:, k);

    % Save sliding variable history for analysis
    s_history(k) = s;

end
u(end) = u(end-1);  % Control signal stabilization for last deste

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Plot %%%%

figure
plot(t, dx(1,:), 'b', 'LineWidth', 2); hold on;
plot(t, dx_hat_L(1,:), '-.g', 'LineWidth', 2); hold on;
plot(t, dx_hat_UDE(1,:), '-r', 'LineWidth', 2); hold on;
plot(t, dx_hat_SMC(1,:), 'y', 'LineWidth', 2); hold on;
plot(t, dx_hat_ST(1,:), 'm', 'LineWidth', 2); hold on;
plot(t, 0, '--k', 'LineWidth', 1.5);
title('Position vs Time');
ylabel('Position (m)');
legend('System Response', 'L System Response', 'UDE System Response', 'SMC System Response', 'ST System Response', 'Reference');
grid on

figure
plot(t, dx(2,:), 'b', 'LineWidth', 2); hold on;
plot(t, dx_hat_L(2,:), '-.g', 'LineWidth', 2); hold on;
plot(t, dx_hat_UDE(2,:), '-r', 'LineWidth', 2); hold on;
plot(t, dx_hat_SMC(2,:), 'y', 'LineWidth', 2); hold on;
plot(t, dx_hat_ST(2,:), 'm', 'LineWidth', 2); hold on;
plot(t, 0, '--k', 'LineWidth', 1.5);
title('Velocity vs Time');
ylabel('Velocity (m/s)');
legend('System Response', 'L System Response', 'UDE System Response', 'SMC System Response', 'ST System Response', 'Reference');
grid on
