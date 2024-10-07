%% Ines Alejandro Garcia Mosqueda
% Ackerman robot simulation with Lyapunov-based control (Extended Model)
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

num_waypoints = size(waypoints, 1);
current_waypoint = 1;

xf = waypoints(current_waypoint, 1);
yf = waypoints(current_waypoint, 2);

%% Control Constants
Kv = 0.5;  % Linear velocity gain
Ktheta = 6.0;  % Gain for steering angle control
Kphi = 10.5;  % Steering angle rate control gain

%% Car Constants
l = 1;  % Length of the car (wheelbase)
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
phi = 0;  % Steering angle
theta = 0;  % Orientation of the vehicle
thetaf = 0;  % Desired orientation
e_x = 0;
e_y = 0;
theta_e = 0;

%% Inputs
V = 0;  % Linear velocity
gamma = 0;  % Steering angle rate

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
    % Error between current position and target waypoint
    e_x = xf - x;
    e_y = yf - y;

    % Distance to the target waypoint
    d = sqrt(e_x^2 + e_y^2);
    % Desired orientation to the waypoint
    thetaf = atan2(e_y, e_x);
    % Orientation error
    theta_e = thetaf - theta;

    % Normalize the orientation error
    if theta_e > pi
        theta_e = theta_e - 2 * pi;
    elseif theta_e < -pi
        theta_e = theta_e + 2 * pi;
    end

    % Compute the angle between vehicle's heading and error vector (alpha)
    alpha = thetaf - theta;

    % Compute linear velocity using Lyapunov-based control law
    %V = Kv * d;  % Negative feedback to drive towards target
    V = Kv * d; 

    % Compute desired steering angle
    if V ~= 0  % Prevent division by zero
        phi_desired = atan2(l * Ktheta * theta_e, V);  % Steering control law
    else
        phi_desired = 0;
    end

    % Limit the desired steering angle
    if phi_desired > deg2rad(max_phi)
        phi_desired = deg2rad(max_phi);
    elseif phi_desired < deg2rad(-max_phi)
        phi_desired = deg2rad(-max_phi);
    end

    % Compute steering rate (gamma) for controlling steering dynamics
    gamma = Kphi * (phi_desired - phi);

    % Update steering angle phi
    phi = phi + gamma * dt;

    % Limit the steering angle after update
    if phi > deg2rad(max_phi)
        phi = deg2rad(max_phi);
    elseif phi < deg2rad(-max_phi)
        phi = deg2rad(-max_phi);
    end 

    % Update kinematics
    x_dot = V * cos(theta);
    y_dot = V * sin(theta);
    theta_dot = V * tan(phi) / l;

    % Update the state variables
    x = x + x_dot * dt;
    y = y + y_dot * dt;
    theta = theta + theta_dot * dt;

    % Normalize theta to [-pi, pi]
    if theta > pi
        theta = theta - 2 * pi;
    elseif theta < -pi
        theta = theta + 2 * pi;
    end

    % Update car visualization
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

    % Record trajectory for plotting
    x_arr = [x_arr, x];
    y_arr = [y_arr, y];
    set(trajectory_line, 'XData', x_arr, 'YData', y_arr);

    % If close to the target, move to the next waypoint
    if d < 0.5
        V = 0;
        gamma = 0;
        if current_waypoint < num_waypoints
            current_waypoint = current_waypoint + 1;
            xf = waypoints(current_waypoint, 1);
            yf = waypoints(current_waypoint, 2);
        else
            break;
        end
    end

    pause(dt/2);
end
