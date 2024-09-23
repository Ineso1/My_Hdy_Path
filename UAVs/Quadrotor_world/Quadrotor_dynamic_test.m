%% Ines Alejandro Garcia Mosqueda
close all;
clearvars; 
clc;

addpath ..\Quadrotor_class\

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Drone Params Instance %%%%

roll = 0;           % Roll (rad)
pitch = 0;          % Pitch (rad)
yaw = pi/4;         % Yaw (45 degrees in rad)

mass = 1.0;         % Mass (kg)
l_propeller = 0.2;  % Distance to propeller (m)
K_T = 1e-4;         % Thrust constant
K_Q = 1e-5;         % Torque constant

drone = QuadrotorNewtonEuler(roll, pitch, yaw, mass, l_propeller, K_T, K_Q);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Simulation Variables %%%%

t_sim = 10;         
dt = 0.1;           
time = 0:dt:t_sim;  

position = zeros(3, length(time));
velocity = zeros(3, length(time));
yaw_angle = zeros(size(time));

position(:, 1) = [0; 0; 0];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Plot Params and Instances %%%%
figure;
hold on;
axis equal;
xlim([-10 10]);
ylim([-10 10]);
zlim([-25 25]);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;

l = 0.5; 
w = 0.3;
h = 0.1; 

drone_vertices = [-l/2, -w/2, -h/2;
                  l/2, -w/2, -h/2;
                  l/2,  w/2, -h/2;
                 -l/2,  w/2, -h/2;
                 -l/2, -w/2,  h/2;
                  l/2, -w/2,  h/2;
                  l/2,  w/2,  h/2;
                 -l/2,  w/2,  h/2];

faces = [1, 2, 3, 4;
         5, 6, 7, 8;
         1, 2, 6, 5;
         2, 3, 7, 6;
         3, 4, 8, 7;
         4, 1, 5, 8];

drone_body = patch('Vertices', drone_vertices, 'Faces', faces, 'FaceColor', 'blue', 'FaceAlpha', 0.5);

thrust_vector = quiver3(0, 0, 0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
orientation_vector = quiver3(0, 0, 0, 0, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Simulation %%%%

for i = 2:length(time)
    yaw_angle(i) = drone.yaw; 
    
    drone = drone.updateState(dt);
    
    acceleration_inertial = drone.translationalDynamics();
    
    velocity(:, i) = velocity(:, i-1) + acceleration_inertial * dt;
    position(:, i) = position(:, i-1) + velocity(:, i) * dt;
    
    drone_position = position(:, i);
    
    drone_orientation = drone.R_bi; 
    
    rotated_vertices = (drone_orientation * drone_vertices')';
    translated_vertices = rotated_vertices + drone_position';

    set(drone_body, 'Vertices', translated_vertices);
    
    thrust_inertial = drone.bodyToInertial([0; 0; drone.thrust]);
    set(thrust_vector, 'XData', drone_position(1), 'YData', drone_position(2), 'ZData', drone_position(3));
    set(thrust_vector, 'UData', thrust_inertial(1), 'VData', thrust_inertial(2), 'WData', thrust_inertial(3));
    
    orientation_inertial = drone.bodyToInertial([1; 0; 0]);
    set(orientation_vector, 'XData', drone_position(1), 'YData', drone_position(2), 'ZData', drone_position(3));
    set(orientation_vector, 'UData', orientation_inertial(1), 'VData', orientation_inertial(2), 'WData', orientation_inertial(3));

    plot3(position(1,1:i), position(2,1:i), position(3,1:i), 'b', 'LineWidth', 1.5);    
    pause(0.1);
end

hold off;
