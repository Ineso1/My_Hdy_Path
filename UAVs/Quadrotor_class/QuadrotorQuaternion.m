%% Ines Alejandro Garcia Mosqueda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef QuadrotorQuaternion

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        properties
            % Drone's constants
            mass            % Drone mass (kg)
            g               % Gravity (m/s^2)
            l_propeller     % Propeller distance to mass center (m)
            K_T             % Propeller thrust constant
            K_Q             % Propeller torque constant 
            J               % Inertia matrix
            
            % Position
            p               % Position vector [x; y; z]
            dp              % Linear velocity vector [dx; dy; dz]
            ddp             % Linear acceleration vector [dxx; ddy; ddz]

            % Orientation Quaternion
            q_prev          % Previous orientation quaternion quaternion(q0, q1, q2, q3)
            q               % Orientation quaternion quaternion(q0, q1, q2, q3)
            omega           % Angular velocity vector [wx; wy; wz]
            domega          % Angular acceleration [dwx; dwy; dwy]

            % Control vars
            thrust          % Total thrust
            
            % Dynamic vectors
            forces          % Body frame forces q * [0; 0; thrust] * q'
            torques         % Body frame torques [tau1; tau2; tau3]

            % Aim Position
            p_aim

            iterations
                
        end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% DroneQuaternion Class Methods %%%%
        methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Constructors %%%%
    
            function obj = QuadrotorQuaternion(q0, q1, q2, q3, mass, l_propeller, K_T, K_Q)
                if nargin >= 4
                    obj.q = quaternion(q0, q1, q2, q3);
                    obj.q_prev = obj.q;
                    
                else
                    obj.q = quaternion(1, 0, 0, 0);
                    obj.q_prev = obj.q;
                end
                if nargin >= 7
                    obj.mass = mass;
                    obj.l_propeller = l_propeller;
                    obj.K_T = K_T;
                    obj.K_Q = K_Q;
                else
                    % Default values
                    obj.mass = 1.0;  
                    obj.l_propeller = 0.1;
                    obj.K_T = 1e-4;
                    obj.K_Q = 1e-5;
                end
    
                obj.g = 9.81;    
                obj.thrust = 0;
                
                % Inertia matrix diagonal considering its symetric
                obj.J = diag([0.01, 0.01, 0.01]);
    
                % Initial position
                obj.p = [0; 0; 0];
                obj.dp = [0; 0; 0];
                obj.ddp = [0; 0; 0];
                
                % Initial angular velocities (rad/s)
                obj.omega = [0; 0; 0]; 
                obj.domega = [0; 0; 0]; 

                % Initial dynamic vars
                obj.forces = [0; 0; 0];
                obj.torques = [0; 0; 0];

                % Initial aim point
                obj.p_aim = [0; 0; 0];
                obj.iterations = 0;
    
            end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Thrust and Torque calc functions %%%%
    
            % Thrust force in body frame
            function thrust_body = calculateThrust(obj)
                % Convert quaternion to 1x4 vector (real first)
                q_vec = compact(obj.q); % Converts quaternion to a 1x4 vector
                thrust_body = quatrotate(q_vec, [0, 0, obj.thrust]); % Rotate thrust vector
            end
                      
    
            % Torque on body frame
            function torque_body = calculateTorque(obj)
                % No rotation for height control for now
                torque_body = [0; 0; 0];
            end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Translational and Rotational Dynamics functions %%%%
    
            % Translational dynamics
            function [acceleration_inertial, velocity_inertial] = translationalDynamics(obj, dt)
                gravity = [0; 0; -obj.mass * obj.g];
                thrust_body = obj.calculateThrust();
                total_force = thrust_body' + gravity;
                acceleration_inertial = total_force / obj.mass;
                velocity_inertial = obj.dp + acceleration_inertial * dt; 
            end
    
            % Rotational dynamics
            function [angular_acceleration_body, angular_velocity_body] = rotationalDynamics(obj, dt)
                torque_body = obj.calculateTorque();            
                angular_acceleration_body = obj.J \ (torque_body - cross(obj.omega, obj.J * obj.omega));
                angular_velocity_body = obj.omega + angular_acceleration_body * dt; 
            end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Update everything on dt time XD %%%%

            % Update the states for each dt
            function obj = updateState(obj, dt)
                % Translational update
                [acc_inertial, vel_inertial] = obj.translationalDynamics(dt);
                obj.dp = vel_inertial;
                obj.p = obj.p + obj.dp * dt + 0.5 * acc_inertial * dt^2;
                
                % Rotational update
                [ang_acc_body, ang_vel_body] = obj.rotationalDynamics(dt);
                obj.omega = obj.omega + ang_acc_body * dt;
                obj.q_prev = obj.q;
                obj.q = obj.q + 0.5 * quaternion(0, obj.omega(1), obj.omega(2), obj.omega(3)) * obj.q * dt; 
                obj.q = normalize(obj.q);
                %disp([' Force: ', num2str(obj.p(3))]);

            end

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Setters %%%%
    
            % Set thrust value
            function obj = setThrust(obj, thrust)
                obj.thrust = thrust;
            end

            % Set target position
            function obj = setAimPoint(obj, target)
                obj.p_aim = target;
            end

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Control law %%%%
            % Simulation update (control law)
            function obj = update_simulation(obj, kp_thrust, kd_thrust, dt)
                height_error = obj.p_aim(3) - obj.p(3);
                obj.thrust = (kp_thrust * height_error - kd_thrust * obj.dp(3));
                obj.torques = [0; 0; 0]; % No rotation, testing only height control
                obj = obj.updateState(dt);
                obj.iterations = obj.iterations + 1;
            end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end     % End methods
    end         % End class
    
