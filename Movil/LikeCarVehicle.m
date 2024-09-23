%% Ines Alejandro Garcia Mosqueda
% Ackerman robot simulation with Lyapunov-based control
%% Init
close all
clear vars
clear

%% Waypoints
waypoints = [
    8, 5;
    -8, 5;
    -10,  7;
    8, -5;
    -8, -5;
    15,  10;
    2,  -4;
    -4,  7;
    2,  6;
    5,  15;
];

%mas puntos
num_waypoints = size(waypoints, 1);
current_waypoint = 1;

xf = waypoints(current_waypoint, 1);
yf = waypoints(current_waypoint, 2);

%% Control Constants
Kv = 0.5; % Linear velocity gain
Kw = 2.0; % Angular velocity gain

%% Car Constants
l = 1;  % Length of the car
w = 0.5;  % Width between wheels

% Constrain
max_phi = 45;  % Max steering angle in degrees

%% Simulation arrays
x_arr = [];
y_arr = [];
phi_arr = [];
theta_arr = [];

tf = 100;  % Simulation time
dt = 0.01;  % Time step
time_simulation = 0:dt:tf;

%% Variables
x = 0;
y = 0;
phi = 0;
theta = 0;
thetaf = 0;
e_x = 0;
e_y = 0;
theta_e = 0;

%% Inputs
V = 0;  % Linear velocity
W = 0;
phi = 0;  % Steering angle

%% Figure
figure;
hold on;
axis equal;
xlim([-15 15]);
ylim([-15 15]);
xlabel('X position');
ylabel('Y position');
title('Car Kinematics');
grid on;

% Plot waypoints
plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
velocity_vector = quiver(x, y, V * cos(theta), V * sin(theta), 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
steering_line = line([x, x + l * cos(theta + phi)], [y, y + l * sin(theta + phi)], 'Color', 'g', 'LineWidth', 2);
car_body = fill([-l/2, l/2, l/2, -l/2], [-w/2, -w/2, w/2, w/2], 'b');
trajectory_line = plot(x, y, 'b-', 'LineWidth', 1.5);

%% Kinematics
for t = time_simulation
    e_x = xf - x;
    e_y = yf - y;

    d = sqrt(e_x^2 + e_y^2);
    thetaf = atan2(e_y, e_x);
    theta_e = thetaf - theta;

    if theta_e > pi
        theta_e = theta_e - 2 * pi;
    elseif theta_e < -pi
        theta_e = theta_e + 2 * pi;
    end

    %V = Kv * (e_x * cos(theta) + e_y * sin(theta)); 
    V = Kv * (e_x * cos(theta) + e_y * sin(theta)); 

    if V < -1
        V = -1;
    end

    W = Kw * theta_e;

    phi = atan2(W * l, V);
    if phi > pi
        phi = phi - 2 * pi;
    elseif phi < -pi
        phi = phi + 2 * pi;
    end

    % Limit the steering angle
    if phi > deg2rad(max_phi)
        phi = deg2rad(max_phi);
    elseif phi < deg2rad(-max_phi)
        phi = deg2rad(-max_phi);
    end

    % If close to the target
    if d < 0.5
        V = 0;
        W = 0;
        % Move to the next waypoint
        if current_waypoint < num_waypoints
            current_waypoint = current_waypoint + 1;
            xf = waypoints(current_waypoint, 1);
            yf = waypoints(current_waypoint, 2);
        else
            break;
        end
    end

    % Update kinematics
    x_dot = V * cos(theta);
    y_dot = V * sin(theta);
    theta_dot = W;

    x = x + x_dot * dt;
    y = y + y_dot * dt;
    theta = theta + theta_dot * dt;

    % Normalize theta to [-pi, pi]
    if theta > pi
        theta = theta - 2 * pi;
    elseif theta < -pi
        theta = theta + 2 * pi;
    end

    car_x = [-l/2, l/2, l/2, -l/2];
    car_y = [-w/2, -w/2, w/2, w/2];
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    rotated_car = R * [car_x; car_y];
    car_x_rot = rotated_car(1, :) + x;
    car_y_rot = rotated_car(2, :) + y;
    
    set(car_body, 'XData', car_x_rot);
    set(car_body, 'YData', car_y_rot);
    set(velocity_vector, 'XData', x, 'YData', y, 'UData', V * cos(theta), 'VData', V * sin(theta));
    set(steering_line, 'XData', [x, x + l * cos(theta + phi)], 'YData', [y, y + l * sin(theta + phi)]);

    x_arr = [x_arr, x];
    y_arr = [y_arr, y];
    set(trajectory_line, 'XData', x_arr, 'YData', y_arr);

    % Animation delay
    pause(dt/2);
end