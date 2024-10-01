classdef Drone < DroneDataExtention
    properties
        g

        mass
        J

        p
        dp
        ddp

        p_d

        ep

        q
        dq
        q_d

        eq
        eq_prev

        omega
        domega

        tau
        F_bf
        u_thrust

        kp_thrust
        kd_thrust_1
        kd_thrust_2

        kp_torque
        kd_torque_1
        kd_torque_2

        max_thrust
        max_torque

        dt
        iterations

        disturbance 

        A
        B
        C
        B_pinv

        Omega_UDE
        xi_UDE
        w_hat_UDE
        x_t_hat_UDE

        x_hat_L
        desired_eigenvalues
        L
        w_hat_L

    end

    methods
        function obj = Drone(mass, Jxx, Jyy, Jzz, q, x0, y0, z0, dt)
            obj@DroneDataExtention();
            if nargin >= 1
                obj.mass = mass;
                obj.J = diag([Jxx, Jyy, Jzz]);
                obj.q = q;
                obj.p = [x0; y0; z0];
                obj.dt = dt;
            else
                obj.mass = 0.5;
                obj.J = diag([5e-3, 5e-3, 1e-2]);
                obj.q = quaternion(1, 0, 0, 0);
                obj.p = [0; 0; 0];
                obj.dt = 0.001;
            end
                obj.g = 9.81;

                obj.dp = [0; 0; 0];
                obj.ddp = [0; 0; 0];
                obj.dq = quaternion(1, 0, 0, 0);

                obj.p_d = [0; 0; 0;];
                obj.q_d = quaternion(1, 0, 0, 0);

                obj.omega = [0; 0; 0;];
                obj.domega = [0; 0; 0;];

                obj.ep = [0; 0; 0;];
                obj.eq = obj.q;

                obj.tau = [0; 0; 0;];
                obj.F_bf = 0;
                obj.u_thrust = [0; 0; 0;];

                obj.kp_thrust = 0;
                obj.kd_thrust_1 = 0;
                obj.kd_thrust_2 = 0;

                obj.kp_torque = 0;
                obj.kd_torque_1 = 0;
                obj.kd_torque_2 = 0;

                obj.max_thrust = 10;
                obj.max_torque = 10;

                obj.iterations = 0;

                obj.disturbance = [0; 0; 0];
                obj = obj.updateDisturbanceArray(obj.disturbance);

                obj.A = [0 0 0 1 0 0;
                         0 0 0 0 1 0;
                         0 0 0 0 0 1;
                         0 0 0 0 0 0;
                         0 0 0 0 0 0;
                         0 0 0 0 0 0;];

                obj.B = [0 0 0; 
                         0 0 0; 
                         0 0 0;
                         1/obj.mass 0 0;
                         0 1/obj.mass 0;
                         0 0 1/obj.mass;];
                
                obj.C = [1 0 0 0 0 0; 
                         0 1 0 0 0 0; 
                         0 0 1 0 0 0];

                obj.B_pinv = pinv(obj.B);

                obj.Omega_UDE = diag([10, 10, 10]);
                x_t = [obj.p; obj.dp]; 
                obj.xi_UDE = -obj.Omega_UDE * (obj.B_pinv * x_t);
                obj.w_hat_UDE = [0; 0; 0;];
                obj.x_t_hat_UDE = [0; 0; 0; 0; 0; 0;];

                obj.desired_eigenvalues = [-5, -5, -5, -10, -10, -10];
                obj.L = place(obj.A', obj.C', obj.desired_eigenvalues)';
                obj.w_hat_L = [0; 0; 0;];
                obj.x_hat_L = [0; 0; 0; 0; 0; 0;];

        end

        function obj = setControlGains(obj, kp_thrust, kd_thrust_1, kd_thrust_2, kp_torque, kd_torque_1, kd_torque_2)
            obj.kp_thrust = kp_thrust;
            obj.kd_thrust_1 = kd_thrust_1;
            obj.kd_thrust_2 = kd_thrust_2;
            obj.kp_torque = kp_torque;
            obj.kd_torque_1 = kd_torque_1;
            obj.kd_torque_2 = kd_torque_2;             
        end

        function obj = setGainUDE(obj, Omega)
            obj.Omega_UDE = diag(Omega);
        end

        function obj = setAimPoint(obj, x_d, y_d, z_d)
            obj.p_d = [x_d; y_d; z_d];
        end

        function obj = updateState(obj)
            
            % dot{X}

            [somethingWeDontCareAbout, ax, ay, az] = parts(obj.q * quaternion( 0, 0, 0, obj.F_bf/obj.mass) * (obj.q'));
            obj.ddp = [ax; ay; az] + obj.disturbance;

            obj.dp = obj.dp + ([0; 0; -obj.g] +  obj.ddp) * obj.dt;

            obj.domega = obj.J\(obj.tau - cross(obj.omega, obj.J * obj.omega));
            
            obj.dq = 0.5*quaternion([0,obj.omega'])*obj.q;

            % X

            obj.p = obj.p + obj.dp * obj.dt + 0.5 * obj.ddp * obj.dt^2;

            obj.q = obj.q + obj.dq * obj.dt;
            obj.q = normalize(obj.q);

            obj.omega = obj.omega + obj.domega * obj.dt;

        end

        function obj = applyControl(obj)
            % Thrust control
            obj = obj.calculateStateUDE();
            obj = obj.calculateStateL();
            obj.ep = obj.p_d - obj.p;
            obj.u_thrust = obj.kp_thrust * obj.ep - obj.kd_thrust_1 * obj.dp;
            if norm(obj.u_thrust)~=0
                obj.u_thrust = obj.max_thrust * tanh(norm(obj.u_thrust) / obj.max_thrust) * obj.u_thrust / norm(obj.u_thrust);
            end
            obj.u_thrust = obj.u_thrust - obj.kd_thrust_2 * obj.dp + [0; 0; obj.mass * obj.g];
            obj.F_bf = norm(obj.u_thrust);

            % Rotation control
            uz_uvec = obj.u_thrust / norm(obj.u_thrust); % unit vector on thrust force direction :)
            obj.q_d = exp(0.5*log(quaternion([dot([0;0;1],uz_uvec);[cross([0;0;1],uz_uvec)]]')));
            obj.q_d = normalize(obj.q_d);

            obj.eq_prev = obj.eq;
            obj.eq = obj.q * (obj.q_d');
            eomega = rotvec(obj.eq * obj.eq_prev')'/obj.dt;

            if norm(rotvec(obj.eq)) > pi || norm(rotvec(obj.eq))< -pi
                obj.q_d = -obj.q_d;
                obj.eq = (obj.q_d') * obj.q;
            end

            u_torque = -obj.kp_torque * rotvec(obj.eq)' - obj.kd_torque_1 * eomega;
            if norm(obj.u_thrust)
                u_torque = obj.max_torque * tanh(norm(u_torque)/obj.max_torque) * u_torque/norm(u_torque) - obj.kd_torque_2 * eomega;
            end
            obj.tau = obj.J * u_torque;

        end

        function obj = updateDroneDataExtention(obj)
            obj.ep_array(:, obj.iterations + 1) = obj.ep;
            obj.eq_array(:, obj.iterations + 1) = rotvec(obj.eq)';
            obj.p_array(:, obj.iterations + 1) = obj.p;
            obj.dp_array(:, obj.iterations + 1) = obj.dp;
            obj.p_d_array(:, obj.iterations + 1) = obj.p_d;
            q_components = compact(obj.q);
            obj.q_array(:, obj.iterations + 1) = q_components;
            q_d_components = compact(obj.q_d);
            obj.q_d_array(:, obj.iterations + 1) = q_d_components;
            obj.omega_array(:, obj.iterations + 1) = obj.omega;
            obj.disturbance_measure_UDE(:, obj.iterations + 1) = obj.w_hat_UDE;
            obj.disturbance_measure_L(:, obj.iterations + 1) = obj.w_hat_L;
            obj.time_array(obj.iterations + 1) = obj.iterations * obj.dt;
            obj = obj.updateDisturbanceArray(obj.disturbance);
        end

        function obj = setDisturbance(obj, disturbance_vector)
            obj.disturbance = disturbance_vector;
        end

        function obj = calculateStateUDE(obj)
            x_t = [obj.p; obj.dp]; 
            xi_dot = -obj.Omega_UDE * obj.xi_UDE - (obj.Omega_UDE^2 * (obj.B_pinv * x_t) + obj.Omega_UDE * (obj.B_pinv * obj.A * x_t)) - obj.Omega_UDE * (obj.u_thrust - [0; 0; obj.g * obj.mass]); 
            obj.xi_UDE = obj.xi_UDE + obj.dt * xi_dot;
            obj.w_hat_UDE = obj.xi_UDE + obj.Omega_UDE * (obj.B_pinv * x_t);  % w_hat(t) = xi + Omega * B^+ x(t)
            dx_hat_UDE = obj.A * x_t + obj.B * obj.u_thrust + obj.B * obj.w_hat_UDE; 
            obj.x_t_hat_UDE = obj.x_t_hat_UDE + obj.dt * dx_hat_UDE;  % Update UDE observer states
        end

        function obj = calculateStateL(obj)
            x_t = [obj.p; obj.dp]; 
            y = obj.C * x_t; 
            dx_hat_L = obj.A * obj.x_hat_L + obj.B * (obj.u_thrust - [0; 0; obj.g * obj.mass]) + obj.L * (y - obj.C * obj.x_hat_L);  
            obj.x_hat_L = obj.x_hat_L + obj.dt * dx_hat_L; 
            obj.w_hat_L = x_t - obj.x_hat_L; 
        end

        function obj = update(obj)
            obj = obj.applyControl();
            obj = obj.updateState();
            obj.iterations = obj.iterations + 1;
            obj = updateDroneDataExtention(obj);
        end
    end
end

