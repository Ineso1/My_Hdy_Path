close all
clearvars
clc
% Define symbolic variables
syms x1 x2 x3 x4 u y C1 C2 real  % system states, input, and output matrix elements
syms l1 l2 l3 l4 real         % observer gain components
syms A11 A12 A21 A22 B1 B2 E1 E2 real  % elements of A, B, and E matrices

% Define the state vector and augmented state vector
x = [x1; x2];
x_a = [x1; x2; x3; x4];  % Augmented state vector (including disturbance and its rate)

% Define A, E, and B matrices for the system
A = [A11, A12; A21, A22];
C = [C1, C2];
E = [E1; E2];
B = [B1; B2];

% Define the augmented system matrices A_a and B_a
A_a = [A, E, [0; 0]; zeros(1, 2), 0, 1; zeros(1, 2), -1, 0];
B_a = [B; 0; 0];

% Define the observer gain matrix L_a
L_a = [l1; l2; l3; l4];

% Define the output matrix for the augmented system
C_a = [C, 0, 0];

% Define the output error (y - C_a * hat{x}_a) in terms of estimation error
syms hat_x1 hat_x2 hat_x3 hat_x4 real  % Estimated states
e_x = [x1 - hat_x1; x2 - hat_x2; 0; 0;];      % State estimation error
output_error = C_a * e_x;                % Output error y - C * hat_x

% Define the estimated augmented state vector
hat_x_a = [hat_x1; hat_x2; hat_x3; hat_x4];  % Estimated augmented state vector

% Define the observer dynamics
dot_hat_x_a = A_a * hat_x_a + B_a * u + L_a * output_error;

% Extract equations for dot_hat_x3 and dot_hat_x4
dot_x3 = dot_hat_x_a(3);
dot_x4 = dot_hat_x_a(4);

% Display the results
disp('The expression for dot_x3 (disturbance estimate dynamics) is:')
disp(simplify(dot_x3))

disp('The expression for dot_x4 (rate of change of disturbance estimate) is:')
disp(simplify(dot_x4))
