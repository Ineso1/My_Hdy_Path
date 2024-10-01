close all
clearvars
clc

Omega = [10 10 50];

mass = 0.5;
Jxx = 5e-3;
Jyy = 5e-3;
Jzz = 1e-2;
q = quaternion(1, 0, 0, 0);
x0 = 0;
y0 = 0;
z0 = 0;
dt = 0.01;
sim_time = 5; % seconds
iterations = sim_time / dt;

drone = Drone(mass, Jxx, Jyy, Jzz, q, x0, y0, z0, dt);
drone = drone.setControlGains(4, 1, 2, 20, 2, 6);
drone = drone.setAimPoint(1, 1, 2);
drone = drone.setDisturbance([0; 0; 0]);
drone = drone.setGainUDE(Omega);


%for i = 1:iterations
    % Apply disturbance between 2 and 3 seconds
%    if i * dt >= 2 && i * dt <= 3
%        drone = drone.setDisturbance([0.5; 1; 6]); % Disturbance in the X direction
%    else
%        drone = drone.setDisturbance([0; 0; 0]); % No disturbance
%    end
%    drone = drone.update();
%end

for i = 1:iterations
    time = i * dt;
    if time >= 2 && time <= 3
        disturbance = [0.5 * sin(pi * (time - 2)); 1 * sin(pi * (time - 2)); 6 * sin(pi * (time - 2))];
        drone = drone.setDisturbance(disturbance);
    else
        drone = drone.setDisturbance([0; 0; 0]); 
    end
    
    drone = drone.update();
end

drone.plotPosition();
drone.plotErrors();
drone.plotOrientation();
drone.plotDisturbanceUDE();
drone.plotDisturbanceL();
pause(3);
drone.animateDroneTrajectory();