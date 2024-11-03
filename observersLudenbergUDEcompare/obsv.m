close all;
clearvars;
clc;

mass = 1;                    % Mass
b = 0.5;                     % Damping
dt = 0.01;                  
t = 0:dt:25;                 
disturbance = 0;           

kp_without_observer = 1;
kd_without_observer = 1.1;
kp_UDE = 1;
kd_UDE = 1.1;
kp_Luenberger = 1;
kd_Luenberger = 1.1;
kp_SuperTwisting = 1.1;
kd_SuperTwisting = 1.1;

desired_eigenvalues = [-7, -8, -10, -12];
Omega = 50;
gamma = 0.1;       
kd_st = 10.5;
lambda = 0.005;
dlambda = 0.1;
k_lambda_0 = 1.5;
k_lambda_1 = 1.1;      
k_disturbance_bound = 10;

model_without_observer = MassModel(mass, b, dt, t);
model_without_observer = model_without_observer.setControlGains(kp_without_observer, kd_without_observer);

model_UDE = MassModel(mass, b, dt, t);
model_UDE = model_UDE.setControlGains(kp_UDE, kd_UDE);
model_UDE = model_UDE.setUDEGain(Omega);

model_Luenberger = MassModel(mass, b, dt, t);
model_Luenberger = model_Luenberger.setControlGains(kp_Luenberger, kd_Luenberger);
model_Luenberger = model_Luenberger.setLudenbergerEign(desired_eigenvalues);

model_SuperTwisting = MassModel(mass, b, dt, t);
model_SuperTwisting = model_SuperTwisting.setControlGains(kp_SuperTwisting, kd_SuperTwisting);
model_SuperTwisting = model_SuperTwisting.setSTGains(lambda, dlambda, k_lambda_0, k_lambda_1, k_disturbance_bound); 

disturbance_start = 5;      
disturbance_end = 15;  
disturbance_start_0 = 10;      
disturbance_end_0 = 14;  
frequency1 = 1;             
frequency2 = 3;              
amplitude1 = 5;              
amplitude2 = 3;              

ref = ones(1, length(t));

model_without_observer.feedback = 1; 
model_UDE.feedback = 1; 
model_Luenberger.feedback = 1;
model_SuperTwisting.feedback = 1;

model_without_observer = model_without_observer.selectObserver(0);
model_UDE = model_UDE.selectObserver(2); 
model_Luenberger = model_Luenberger.selectObserver(1); 
model_SuperTwisting = model_SuperTwisting.selectObserver(5); 
model_without_observer.x = [10; 10];
model_UDE.x = [10; 10];
model_Luenberger.x = [10; 10; 0; 0;];
model_SuperTwisting.x = [10; 10];


applied_disturbance = zeros(1, length(t));
for k = 1:length(t)
    if t(k) >= disturbance_start && t(k) <= disturbance_end
        disturbance = amplitude1 * sin(2 * pi * frequency1 * t(k)) + amplitude2 * sin(2 * pi * frequency2 * t(k));
    else
        disturbance = 0;
    end
    if t(k) >= disturbance_start_0 && t(k) <= disturbance_end_0
        disturbance = disturbance + amplitude1 * sin(2 * pi * frequency1+1 * t(k)) + amplitude2 * sin(2 * pi * frequency2+2 * t(k));
    end
    applied_disturbance(k) = disturbance;

    model_without_observer = model_without_observer.setDisturbance(disturbance);
    model_without_observer = model_without_observer.update(ref(k));

    model_UDE = model_UDE.setDisturbance(disturbance);
    model_UDE = model_UDE.update(ref(k));

    model_Luenberger = model_Luenberger.setDisturbance(disturbance);
    model_Luenberger = model_Luenberger.update(ref(k));

    model_SuperTwisting = model_SuperTwisting.setDisturbance(disturbance);
    model_SuperTwisting = model_SuperTwisting.update(ref(k));
end

% Plotting results with comparisons
figure;

% Position Comparison
subplot(4, 2, [1 2]);
plot(t, model_without_observer.x_arr(1, :), 'b', 'LineWidth', 1.5); hold on;
plot(t, model_UDE.x_arr(1, :), 'r--', 'LineWidth', 1.5);
plot(t, model_Luenberger.x_arr(1, :), 'g-.', 'LineWidth', 1.5);
plot(t, model_SuperTwisting.x_arr(1, :), 'm:', 'LineWidth', 1.5);
title('Position Comparison');
ylabel('Position (m)');
legend('No observer', 'UDE', 'Luenberger', 'Super-Twisting');
grid on;

% Velocity Comparison
subplot(4, 2, 3);
plot(t, model_without_observer.dx_arr(1, :), 'b', 'LineWidth', 1.5); hold on;
plot(t, model_UDE.dx_arr(1, :), 'r--', 'LineWidth', 1.5);
plot(t, model_Luenberger.dx_arr(1, :), 'g-.', 'LineWidth', 1.5);
plot(t, model_SuperTwisting.dx_arr(1, :), 'm:', 'LineWidth', 1.5);
title('Velocity Comparison');
ylabel('Velocity (m)');
legend('No observer', 'UDE', 'Luenberger', 'Super-Twisting');
grid on;

% Acceleration Comparison
subplot(4, 2, 4);
plot(t, model_without_observer.dx_arr(2, :), 'b', 'LineWidth', 1.5); hold on;
plot(t, model_UDE.dx_arr(2, :), 'r--', 'LineWidth', 1.5);
plot(t, model_Luenberger.dx_arr(2, :), 'g-.', 'LineWidth', 1.5);
plot(t, model_SuperTwisting.dx_arr(2, :), 'm:', 'LineWidth', 1.5);
title('Acceleration Comparison');
ylabel('Acceleration (m/s^2)');
legend('No observer', 'UDE', 'Luenberger', 'Super-Twisting');
grid on;

% Control Input u Comparison
subplot(4, 2, [5 6]);
plot(t, model_without_observer.u_arr, 'b', 'LineWidth', 1.5); hold on;
plot(t, model_UDE.u_arr, 'r--', 'LineWidth', 1.5);
plot(t, model_Luenberger.u_arr, 'g-.', 'LineWidth', 1.5);
plot(t, model_SuperTwisting.u_arr, 'm:', 'LineWidth', 1.5);
title('Control Input u');
xlabel('Time (s)');
ylabel('Control Input u');
legend('No observer', 'UDE', 'Luenberger', 'Super-Twisting');
grid on;

% Disturbance Estimate w_hat Comparison
subplot(4, 2, [7 8]);
plot(t, applied_disturbance, 'k', 'LineWidth', 1.5); hold on;
plot(t, model_without_observer.w_hat_arr, 'b', 'LineWidth', 1.5); hold on;
plot(t, model_UDE.w_hat_arr, 'r--', 'LineWidth', 1.5);
plot(t, model_Luenberger.w_hat_arr, 'g-.', 'LineWidth', 1.5);
plot(t, model_SuperTwisting.w_hat_arr, 'm:', 'LineWidth', 1.5);
title('Disturbance Estimate w');
xlabel('Time (s)');
ylabel('Disturbance Estimate');
legend('Applied Disturbance', 'No observer', 'UDE', 'Luenberger', 'Super-Twisting');
grid on;

% Error Comparison
error_without_observer = ref - model_without_observer.x_arr(1, :);
error_UDE = ref - model_UDE.x_arr(1, :);
error_Luenberger = ref - model_Luenberger.x_arr(1, :);
error_SuperTwisting = ref - model_SuperTwisting.x_arr(1, :);

figure;
plot(t, error_without_observer, 'b', 'LineWidth', 1.5); hold on;
plot(t, error_UDE, 'r--', 'LineWidth', 1.5);
plot(t, error_Luenberger, 'g-.', 'LineWidth', 1.5);
plot(t, error_SuperTwisting, 'm:', 'LineWidth', 1.5);
title('Error Comparison (UDE vs. Luenberger vs. Super-Twisting vs. No Observer)');
xlabel('Time (s)');
ylabel('Error (Position - Reference)');
legend('No observer', 'UDE', 'Luenberger', 'Super-Twisting');
grid on;
