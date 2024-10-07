close all
clearvars
clc

Omega_trans = [20 20 70];
Omega_rot = [1 1 1];

mass = 1.5;
Jxx = 5e-3;
Jyy = 5e-3;
Jzz = 1e-2;
q = quaternion(1, 0, 0, 0);
x0 = 0;
y0 = 0;
z0 = 0;
dt = 0.01;
sim_time = 8; % seconds
iterations = sim_time / dt;

drone = Drone(mass, Jxx, Jyy, Jzz, q, x0, y0, z0, dt);
drone = drone.setControlGains(4, 1, 2, 20, 2, 6);
drone = drone.setAimPoint(1, 1, 2);
drone = drone.setDisturbance([0; 0; 0], [0; 0; 0]);
drone = drone.setGainUDE(Omega_trans, Omega_rot);


disturbance_trans = [0; 0; 0];
disturbance_rot = [0; 0; 0];

for i = 1:iterations
    time = i * dt;
    if time >= 2 && time <= 3
        disturbance_trans = [0.5 * sin(pi * (time - 2)); 1 * sin(pi * (time - 2)); 6 * sin(pi * (time - 2))];
        disturbance_rot = [3.5 * sin(pi * (time - 2)); 2.5 * sin(pi * (time - 2)); 6.5 * sin(pi * (time - 2))];
        drone = drone.setDisturbance(disturbance_trans, disturbance_rot);
    else
        drone = drone.setDisturbance([0; 0; 0], [0; 0; 0]); 
    end
    
    drone = drone.update();
end

%drone.plotPosition();
%drone.plotErrors();
%drone.plotOrientation();
drone.plotDisturbanceUDE_trans();
drone.plotDisturbanceUDE_rot();
%drone.plotDisturbanceL_trans();
%drone.plotDisturbanceL_rot();
drone.plotObserverStates_trans();
drone.plotObserverStates_rot();
pause(3);
drone.animateDroneTrajectory();