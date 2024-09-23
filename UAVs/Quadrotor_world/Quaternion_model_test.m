%% Ines Alejandro Garcia Mosqueda
addpath ..\Quadrotor_class\

close all
clearvars
clc

drone = QuadrotorQuaternion(1, 0, 0, 0, 1.5, 0.2, 1e-4, 1e-5);
drone = drone.setAimPoint([0, 0, 10]);

dt = 0.01;
kp_thrust = 10;
kd_thrust = 2;
sim_time = 10; % seconds
time_steps = 0:dt:sim_time;

height = zeros(length(time_steps), 1);
orientation = zeros(length(time_steps), 4); % Quaternion components (q0, q1, q2, q3)
forces = zeros(length(time_steps), 3); % Forces on x, y, z axes
torques = zeros(length(time_steps), 3); % Torques on x, y, z axes
height_error = zeros(length(time_steps), 1);

for i = 1:length(time_steps)
    t = time_steps(i);
    drone = drone.update_simulation(kp_thrust, kd_thrust, dt);
    height(i) = drone.p(3); % Height (z-axis)
    orientation(i, :) = compact(drone.q); % Quaternion (orientation)
    forces(i, :) = [0, 0, drone.thrust]; % Thrust force
    torques(i, :) = drone.torques; % Torques
    height_error(i) = drone.p_aim(3) - drone.p(3); % Height error
    
end

figure;

subplot(3,2,1);
plot(time_steps, height);
title('Height (z-axis)');
xlabel('Time (s)');
ylabel('Height (m)');

subplot(3,2,2);
plot(time_steps, orientation(:, 1), 'r', time_steps, orientation(:, 2), 'g', ...
     time_steps, orientation(:, 3), 'b', time_steps, orientation(:, 4), 'k');
title('Orientation (Quaternion)');
xlabel('Time (s)');
ylabel('Quaternion components');
legend('q0', 'q1', 'q2', 'q3');

subplot(3,2,3);
plot(time_steps, forces(:, 1), 'r', time_steps, forces(:, 2), 'g', time_steps, forces(:, 3), 'b');
title('Forces');
xlabel('Time (s)');
ylabel('Force (N)');
legend('Force X', 'Force Y', 'Force Z');

subplot(3,2,4);
plot(time_steps, torques(:, 1), 'r', time_steps, torques(:, 2), 'g', time_steps, torques(:, 3), 'b');
title('Torques');
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('Torque X', 'Torque Y', 'Torque Z');

subplot(3,2,5);
plot(time_steps, height_error);
title('Height Error');
xlabel('Time (s)');
ylabel('Error (m)');

