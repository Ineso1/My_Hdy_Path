%% Ines Alejandro Garcia Mosqueda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef QuadrotorQuaternionPD

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        properties
            % Drone's constants
            mass            % Drone mass (kg)
            g               % Gravity (m/s^2)
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
            omegae           
            qe              % Orientation error quaternion quaternion(q0, q1, q2, q3)
            qe_prev         % Orientation previous error quaternion quaternion(q0, q1, q2, q3)
            qd              % Desire orientation
        

            % Control vars
            thrust          % Total thrust
            
            % Dynamic vectors
            forces          % Body frame forces q * [0; 0; thrust] * q'
            torques         % Body frame torques [tau1; tau2; tau3]

            % Aim Position
            p_aim
            q_aim

            % PD Control gains (set through setters)
            kp_thrust
            kd_thrust_1
            kd_thrust_2

            kp_rotation
            kd_rotation_1
            kd_rotation_2

            lambda1 
            lambda2

            max_thrust
            max_rotation

            % Just for debug :p
            iterations
                
        end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% DroneQuaternion Class Methods %%%%
        methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Constructors %%%%
            function obj = QuadrotorQuaternionPD(q0, q1, q2, q3, mass)
                if nargin >= 3
                    obj.q = quaternion(q0, q1, q2, q3);
                    obj.q_prev = obj.q;
                else
                    obj.q = quaternion(1, 0, 0, 0);
                    obj.q_prev = obj.q;
                end
                
                if nargin >= 4
                    obj.mass = mass;
                else
                    obj.mass = 0.5;  
                end
                
                obj.g = 9.81;    
                obj.thrust = 0;
                obj.J = diag([5e-3, 5e-3, 1e-2]); % Inertia matrix
                
                % Initial position
                obj.p = [0; 0; 0];
                obj.dp = [0; 0; 0];
                obj.ddp = [0; 0; 0];
                
                % Initial angular velocities (rad/s)
                obj.q = quaternion(1,0,0,0);
                obj.omega = [0; 0; 0]; 
                obj.domega = [0; 0; 0];

                obj.omegae = obj.omega;
                
                obj.qe = obj.q;
                obj.qd = obj.q;
                
                % Initial control variables
                obj.forces = [0; 0; 0];
                obj.torques = [0; 0; 0];
                
                obj.p_aim = [0; 0; 0];
                obj.q_aim = quaternion(1, 0, 0, 0); % Default orientation goal
                
                obj.max_thrust = 9.5;
                obj.max_rotation = 10;

                obj.kp_thrust = 0;
                obj.kd_thrust_1 = 0;
                obj.kd_thrust_2 = 0;

                obj.kp_rotation = 0;
                obj.kd_rotation_1 = 0;
                obj.kd_rotation_2 = 0;
                
                obj.lambda1 = 0;
                obj.lambda2 = 0;

                obj.iterations = 0;

            end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Setters %%%%

            function obj = setControlGains(obj, kp_thrust, kd_thrust_1, kd_thrust_2, kp_rotation, kd_rotation_1, kd_rotation_2, lambda1, lambda2)
                obj.kp_thrust = kp_thrust;
                obj.kd_thrust_1 = kd_thrust_1;
                obj.kd_thrust_2 = kd_thrust_2;

                obj.kp_rotation = kp_rotation;
                obj.kd_rotation_1 = kd_rotation_1;
                obj.kd_rotation_2 = kd_rotation_2;
                
                obj.lambda1 = lambda1;
                obj.lambda2 = lambda2;
            end

            % Set thrust value
            function obj = setThrust(obj, thrust)
                obj.thrust = thrust;
            end

            % Set target position
            function obj = setAimPoint(obj, target)
                obj.p_aim = target;
            end

            % Set target quaternion
            function obj = setAimOrientation(obj, target_q)
                obj.q_aim = target_q;
            end

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Thrust and Torque calc functions %%%%
    
            % Thrust force in body frame
            function thrust_body = calculateThrust(obj)
                q_vec = compact(obj.q); % Convert quaternion to vector form
                thrust_body = quatrotate(q_vec, [0, 0, obj.thrust]);
            end
                      
    
            % Torque on body frame
            function torque_body = calculateTorque(obj)
                torque_body = obj.torques; % Rotational torques calculated during control
            end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Translational and Rotational Dynamics functions %%%%
    
            % Translational dynamics
            function [acceleration_inertial, velocity_inertial] = translationalDynamics(obj, dt)
                [some, ax, ay, az] = parts(obj.q * quaternion( 0, 0, 0, obj.thrust/obj.mass) * (obj.q'));
                acceleration_inertial = [ax; ay; az];
                velocity_inertial = obj.dp + ([0; 0; -obj.g] +  [ax; ay; az])* dt;
            end
    
            % Rotational dynamics
            function [angular_acceleration_body, angular_velocity_body] = rotationalDynamics(obj, dt)
                angular_velocity_body = 0.5*quaternion([0,obj.omega'])*obj.q;
                angular_acceleration_body = obj.J \ (obj.torques- cross(obj.omega,obj.J * obj.omega));
            end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Control %%%%

            function obj = applyControl(obj, dt)
                % Translational control law (PD control for thrust)
                pos_error = obj.p_aim - obj.p;
                thrust_pd = obj.kp_thrust * pos_error - obj.kd_thrust_1 * obj.dp;
                if norm(thrust_pd)~=0
                    thrust_pd = obj.max_thrust * tanh(norm(thrust_pd)/obj.max_thrust) * thrust_pd/norm(thrust_pd);
                end
                thrust_pd = thrust_pd - obj.kd_thrust_2 * obj.dp + [0; 0; obj.mass * obj.g];
                
                obj.thrust = norm(thrust_pd);
    
                Fu = [0;0;0];
                if obj.thrust ~= 0
                    Fu = thrust_pd/obj.thrust;
                    obj.qd = exp(0.5*log(quaternion([dot([0;0;1],Fu);[cross([0;0;1],Fu)]]')));
                    obj.qd = normalize(obj.qd);
                end
    
                obj.qe_prev = obj.qe;
                obj.qe = obj.q*(obj.qd');
    
                obj.omegae = rotvec(obj.qe * obj.qe_prev')'/dt;
                if norm(rotvec(obj.qe)) > pi || norm(rotvec(obj.qe))< -pi
                    obj.qd = -obj.qd;
                    obj.qe = (obj.qd') * obj.q;
                end
    
                % Rotational control law (PD control for orientation)
                ur = -obj.kp_rotation * rotvec(obj.qe)' - obj.kd_rotation_1 * obj.omegae ;
                if norm(ur)~=0
                    ur = obj.max_rotation * tanh(norm(ur)/obj.max_rotation) * ur/norm(ur) - obj.kd_rotation_2 * obj.omegae  ;
                end
                obj.torques = obj.J * ur; % Convert control signal to torques
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
                obj.domega = ang_vel_body;
                obj.q = obj.q + ang_vel_body * dt;
                obj.q = normalize(obj.q);
                obj.omega = obj.omega + ang_acc_body * dt;
            end

            function obj = update_simulation(obj, dt)
                obj = obj.applyControl(dt);  % Apply control law
                obj = obj.updateState(dt);   % Update the state
                obj.iterations = obj.iterations + 1;
            end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end     % End methods
    end         % End class
    
