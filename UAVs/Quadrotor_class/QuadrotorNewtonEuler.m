%% Ines Alejandro Garcia Mosqueda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef QuadrotorNewtonEuler

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        properties
            % Drone's constants
            mass            % Drone mass (kg)
            g               % Gravity (m/s^2)
            l_propeller     % Propeller distance to mass center (m)
            K_T             % Propeller thrust constant
            K_Q             % Propeller torque constant 
            J               % Inertia matrix
            W_n             % Angular velocity convertion
            
            % Euler angles
            roll            % Roll angle (phi)
            pitch           % Pitch angle (theta)
            yaw             % Yaw angle (psi)
    
            % Control varss    
            thrust          % Thrust forces [0; 0; Thrust]
            
            % Rotational velocities 
            omega           % Angular velocity vector [wx; wy; wz] in body frame
    
            % Rotation matrices
            R_bi            % Body to inertial rotation matrix
            R_ib            % Inertial to body rotation matrix
        end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% DroneQuaternion Class Methods %%%%
        methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Constructors %%%%
    
            function obj = QuadrotorNewtonEuler(roll, pitch, yaw, mass, l_propeller, K_T, K_Q)
                if nargin >= 3
                    obj.roll = roll;
                    obj.pitch = pitch;
                    obj.yaw = yaw;
                else
                    obj.roll = 0;
                    obj.pitch = 0;
                    obj.yaw = 0;
                end
                if nargin >= 6
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
    
                % Initial angular velocities (rad/s)
                obj.omega = [0; 0; 0]; 
    
                obj = obj.updateRotationMatrices();
            end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Rotaton functions %%%%
    
            % Update the rotation matrices
            function obj = updateRotationMatrices(obj)
                R_roll = [1 0 0;
                          0 cos(obj.roll) -sin(obj.roll);
                          0 sin(obj.roll) cos(obj.roll)];
    
                R_pitch = [cos(obj.pitch) 0 sin(obj.pitch);
                           0 1 0;
                           -sin(obj.pitch) 0 cos(obj.pitch)];
    
                R_yaw = [cos(obj.yaw) -sin(obj.yaw) 0;
                         sin(obj.yaw) cos(obj.yaw) 0;
                         0 0 1];
                     
                % Rotation matrix from body to inertial
                obj.R_bi = R_yaw * R_pitch * R_roll;
                
                % Rotation matrix from inertial to body
                obj.R_ib = obj.R_bi';  % Transpose of R_bi
    
                obj.W_n = [[1 0 -sin(obj.pitch)],
                [0 cos(obj.roll) sin(obj.roll)*cos(obj.pitch)],
                [0 -sin(obj.roll) cos(obj.roll)*cos(obj.pitch)]];
            end
            
            % Vector from body frame to inertial frame
            function vec_inertial = bodyToInertial(obj, vec_body)
                vec_inertial = obj.R_bi * vec_body;
            end
    
            % Vector from inertial frame to body frame
            function vec_body = inertialToBody(obj, vec_inertial)
                vec_body = obj.R_ib * vec_inertial;
            end
    
            % Update the Euler angles and rotation matrices
            function obj = updateEulerAngles(obj, roll, pitch, yaw)
                obj.roll = roll;
                obj.pitch = pitch;
                obj.yaw = yaw;
                obj = obj.updateRotationMatrices();
            end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Thrust and Torque calc functions %%%%
    
            % Thrust force in body frame
            function thrust_body = calculateThrust(obj)
                thrust_body = [0; 0; 10 * obj.mass]; 
            end
    
            % Torque on body frame
            function torque_body = calculateTorque(obj)
                torque_body = [0; 0.000001; 0.01];
            end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Translational and Rotational Dynamics functions %%%%
    
            % Translational dynamics
            function acceleration_inertial = translationalDynamics(obj)
                gravity = [0; 0; -obj.mass * obj.g];
                thrust_body = obj.calculateThrust();
                thrust_inertial = obj.bodyToInertial(thrust_body);
                total_force = thrust_inertial + gravity;
                acceleration_inertial = total_force / obj.mass;
            end
    
            % Rotational dynamics
            function angular_acceleration_body = rotationalDynamics(obj)
                torque_body = obj.calculateTorque();            
                angular_acceleration_body = obj.J \ (torque_body - cross(obj.omega, obj.J * obj.omega));
            end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Update everything on dt time XD %%%%
    
            % Update the state (omega, angles) for each dt
            function obj = updateState(obj, dt)
                angular_acceleration = obj.rotationalDynamics();
                obj.omega = obj.omega + angular_acceleration * dt;
                euler_rates = obj.W_n \ obj.omega;
                obj.roll = obj.roll + euler_rates(1) * dt;
                obj.pitch = obj.pitch + euler_rates(2) * dt;
                obj.yaw = obj.yaw + euler_rates(3) * dt;
                obj = obj.updateRotationMatrices();
            end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Setters %%%%
    
            function obj = update_Thrust(obj, thrust)
                obj.thrust = thrust;
            end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end     % End methods
    end         % End class
    