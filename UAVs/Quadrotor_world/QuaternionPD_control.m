addpath ..\Quadrotor_class\

close all
clearvars
clc

drone = QuadrotorQuaternionPD(1, 0, 0, 0, 0.5);
aim_point = [10; 10; 10];  % Aim point [x; y; z]
drone = drone.setAimPoint(aim_point);
drone = drone.setControlGains(4, 1, 2, 20, 2, 6, 0.1, 0.1);

dt = 0.01;
sim_time = 10;
time_steps = 0:dt:sim_time;

position = zeros(length(time_steps), 3);
orientation = zeros(length(time_steps), 4);
forces = zeros(length(time_steps), 3);
torques = zeros(length(time_steps), 3);
position_error = zeros(length(time_steps), 3);

% Ainimation plot set
figure('Name', 'Drone 3D Animation');
axis([-20 20 -20 20 0 20]); % limits x, y, z
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
grid on;
hold on;

% Plot aim point
plot3(aim_point(1), aim_point(2), aim_point(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Aim point marker

% Drone marker and trajectory
drone_plot = plot3(0, 0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % Drone marker
trajectory_plot = plot3(nan, nan, nan, 'r-', 'LineWidth', 1); % Path trajectory

% isometric view
view(3);
axis vis3d;

% Simulation
for i = 1:length(time_steps)
    drone = drone.update_simulation(dt);
    
    position(i, :) = drone.p';
    orientation(i, :) = compact(drone.q);
    forces(i, :) = [0, 0, drone.thrust];
    torques(i, :) = drone.torques;
    position_error(i, :) = drone.p_aim - drone.p;

    % Update position on the plot
    set(drone_plot, 'XData', drone.p(1), 'YData', drone.p(2), 'ZData', drone.p(3));
    set(trajectory_plot, 'XData', position(1:i, 1), 'YData', position(1:i, 2), 'ZData', position(1:i, 3));

    pause(0.01);
end

plot3(position(:, 1), position(:, 2), position(:, 3), 'r-', 'LineWidth', 1.5);
hold off;

figure;

subplot(3,2,1);
plot(time_steps, position(:, 3));
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
plot(time_steps, position_error(:, 3));
title('Height Error (z-axis)');
xlabel('Time (s)');
ylabel('Error (m)');

subplot(3,2,6);
plot(time_steps, position_error(:, 1), 'r', time_steps, position_error(:, 2), 'g', time_steps, position_error(:, 3), 'b');
title('Position Error (X, Y, Z)');
xlabel('Time (s)');
ylabel('Position Error (m)');
legend('Error X', 'Error Y', 'Error Z');
