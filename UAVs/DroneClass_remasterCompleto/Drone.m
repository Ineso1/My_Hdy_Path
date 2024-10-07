classdef Drone < DroneDataExtention
    properties
        g
        mass
        J

        % Translational dynamics
        p
        dp
        ddp
        p_d
        ep

        % Rotational dynamics
        q
        dq
        q_d
        eq
        eq_prev
        omega
        domega
        tau

        % Control variables
        F_bf
        u_thrust
        u_torque
        kp_thrust
        kd_thrust_1
        kd_thrust_2
        kp_torque
        kd_torque_1
        kd_torque_2
        max_thrust
        max_torque

        % State space estimation
        dx_sys_trans
        dx_sys_rot
        dx_hat_UDE_trans
        dx_hat_UDE_rot
        dx_hat_L_trans
        dx_hat_L_rot

        % General simulation variables
        dt
        iterations
        disturbance_trans
        disturbance_rot

        %% Translational observer variables (Luenberger and UDE)
        A_trans
        B_trans
        C_trans
        B_pinv_trans
        Omega_UDE_trans
        xi_UDE_trans
        w_hat_UDE_trans
        x_hat_UDE_trans

        x_hat_L_trans
        desired_eigenvalues_trans
        L_trans
        w_hat_L_trans

        %% Rotational observer variables (Luenberger and UDE)
        A_rot
        B_rot
        C_rot
        B_pinv_rot
        Omega_UDE_rot
        xi_UDE_rot
        w_hat_UDE_rot
        x_hat_UDE_rot

        x_hat_L_rot
        desired_eigenvalues_rot
        L_rot
        w_hat_L_rot

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
            obj.iterations = 0;
            
            % State space init
            obj.dx_sys_trans = [0; 0; 0; 0; 0; 0;];
            obj.dx_sys_rot = [0; 0; 0; 0; 0; 0;];
            obj.dx_hat_UDE_trans = [0; 0; 0; 0; 0; 0;];
            obj.dx_hat_UDE_rot = [0; 0; 0; 0; 0; 0;];
            obj.dx_hat_L_trans = [0; 0; 0; 0; 0; 0;];
            obj.dx_hat_L_rot = [0; 0; 0; 0; 0; 0;];

            % Gravity and disturbances
            obj.g = 9.81;
            obj.disturbance_trans = [0; 0; 0];
            obj.disturbance_rot = [0; 0; 0];

            % Initialize control parameters
            obj.F_bf = 0;
            obj.u_thrust = [0; 0; 0];
            obj.u_torque = [0; 0; 0];
            obj.kp_thrust = 0;
            obj.kd_thrust_1 = 0;
            obj.kd_thrust_2 = 0;
            obj.kp_torque = 0;
            obj.kd_torque_1 = 0;
            obj.kd_torque_2 = 0;
            obj.max_thrust = 10;
            obj.max_torque = 10;

            % Initialize observer matrices for translational dynamics (Luenberger and UDE)
            obj.A_trans = [zeros(3,3), eye(3); zeros(3,6)];  % 6x6 matrix
            obj.B_trans = [zeros(3,3); eye(3)/obj.mass];     % 6x3 matrix
            obj.C_trans = [eye(3), zeros(3,3)];              % 3x6 matrix
            obj.B_pinv_trans = pinv(obj.B_trans);            % Pseudo-inverse of B_trans
            obj.Omega_UDE_trans = diag([10, 10, 10]);
            x_t = [obj.p; obj.dp];
            obj.xi_UDE_trans = -obj.Omega_UDE_trans * (obj.B_pinv_trans * x_t);
            obj.w_hat_UDE_trans = zeros(3,1);
            obj.x_hat_UDE_trans = zeros(6,1);
            obj.desired_eigenvalues_trans = [-10, -10, -10, -15, -15, -15];
            obj.L_trans = place(obj.A_trans', obj.C_trans', obj.desired_eigenvalues_trans)';
            obj.w_hat_L_trans = [0; 0; 0;];
            obj.x_hat_L_trans = [0; 0; 0; 0; 0; 0;];

            % Initialize observer matrices for rotational dynamics (Luenberger and UDE)
            obj.A_rot = [zeros(3,3), eye(3); zeros(3,6)];    % 6x6 matrix
            obj.B_rot = [zeros(3,3); inv(obj.J)];            % 6x3 matrix
            obj.C_rot = [eye(3), zeros(3,3)];                % 3x6 matrix
            obj.B_pinv_rot = pinv(obj.B_rot);                % Pseudo-inverse of B_rot
            obj.Omega_UDE_rot = diag([10, 10, 10]);
            [q_vec_0, q_vec_1, q_vec_2, q_vec_3] = parts(obj.q);
            x_r = [q_vec_1; q_vec_2; q_vec_3; obj.omega];
            obj.xi_UDE_rot = -obj.Omega_UDE_rot * (obj.B_pinv_rot * x_r);
            obj.w_hat_UDE_rot = zeros(3,1);
            obj.x_hat_UDE_rot = zeros(6,1);
            obj.desired_eigenvalues_rot = [-15, -15, -15, -20, -20, -20];
            obj.L_rot = place(obj.A_rot', obj.C_rot', obj.desired_eigenvalues_rot)';
            obj.w_hat_L_rot = [0; 0; 0;];
            obj.x_hat_L_rot = [0; 0; 0; 0; 0; 0;];

            obj = obj.updateDisturbanceArray(obj.disturbance_trans, obj.disturbance_rot);
        end

        function obj = setControlGains(obj, kp_thrust, kd_thrust_1, kd_thrust_2, kp_torque, kd_torque_1, kd_torque_2)
            obj.kp_thrust = kp_thrust;
            obj.kd_thrust_1 = kd_thrust_1;
            obj.kd_thrust_2 = kd_thrust_2;
            obj.kp_torque = kp_torque;
            obj.kd_torque_1 = kd_torque_1;
            obj.kd_torque_2 = kd_torque_2;             
        end

        function obj = setGainUDE(obj, Omega_trans, Omega_rot)
            obj.Omega_UDE_trans = diag(Omega_trans);
            obj.Omega_UDE_rot = diag(Omega_rot);
        end

        function obj = setAimPoint(obj, x_d, y_d, z_d)
            obj.p_d = [x_d; y_d; z_d];
        end

        function obj = updateState(obj)
            
            % dot{X}

            [somethingWeDontCareAbout, ax, ay, az] = parts(obj.q * quaternion( 0, 0, 0, obj.F_bf/obj.mass) * (obj.q'));
            obj.ddp = [ax; ay; az] + obj.disturbance_trans;

            obj.dp = obj.dp + ([0; 0; -obj.g] +  obj.ddp) * obj.dt;

            obj.domega = obj.J\(obj.tau - cross(obj.omega, obj.J * obj.omega)) + obj.disturbance_rot;
            
            obj.dq = 0.5*quaternion([0,obj.omega'])*obj.q;

            obj.dx_sys_trans = [obj.dp; obj.ddp - obj.disturbance_trans];
            
            [dq_vec_0, dq_vec_1, dq_vec_2, dq_vec_3] = parts(obj.dq);
            obj.dx_sys_rot = [dq_vec_1; dq_vec_2; dq_vec_3; obj.domega - obj.disturbance_rot];

            % X

            obj.p = obj.p + obj.dp * obj.dt + 0.5 * obj.ddp * obj.dt^2;

            obj.q = obj.q + obj.dq * obj.dt;
            obj.q = normalize(obj.q);

            obj.omega = obj.omega + obj.domega * obj.dt;

        end

        function obj = applyControl(obj)
            % Thrust control
            obj = obj.calculateStateUDE_trans();
            obj = obj.calculateStateUDE_rot();
            obj = obj.calculateStateL_trans();
            obj = obj.calculateStateL_rot();
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

            obj.u_torque = -obj.kp_torque * rotvec(obj.eq)' - obj.kd_torque_1 * eomega;
            if norm(obj.u_thrust)
                obj.u_torque = obj.max_torque * tanh(norm(obj.u_torque)/obj.max_torque) * obj.u_torque/norm(obj.u_torque) - obj.kd_torque_2 * eomega;
            end
            obj.tau = obj.J * obj.u_torque;

        end

        function obj = updateDroneDataExtention(obj)
            obj.ep_array(:, obj.iterations + 1) = obj.ep;
            obj.eq_array(:, obj.iterations + 1) = rotvec(obj.eq)';
            obj.p_array(:, obj.iterations + 1) = obj.p;
            obj.dp_array(:, obj.iterations + 1) = obj.dp;
            obj.ddp_array(:, obj.iterations + 1) = obj.ddp;
            obj.p_d_array(:, obj.iterations + 1) = obj.p_d;
            q_components = compact(obj.q);
            obj.q_array(:, obj.iterations + 1) = q_components;
            dq_components = compact(obj.dq);
            obj.dq_array(:, obj.iterations + 1) = dq_components;
            q_d_components = compact(obj.q_d);
            obj.q_d_array(:, obj.iterations + 1) = q_d_components;
            obj.omega_array(:, obj.iterations + 1) = obj.omega;
            obj.domega_array(:, obj.iterations + 1) = obj.domega;
            obj.disturbance_measure_UDE_trans(:, obj.iterations + 1) = obj.w_hat_UDE_trans;
            obj.disturbance_measure_L_trans(:, obj.iterations + 1) = obj.w_hat_L_trans;
            obj.disturbance_measure_UDE_rot(:, obj.iterations + 1) = obj.w_hat_UDE_rot;
            obj.disturbance_measure_L_rot(:, obj.iterations + 1) = obj.w_hat_L_rot;
            obj.dx_stateUDE_trans(:, obj.iterations + 1) = obj.dx_hat_UDE_trans;
            obj.dx_stateL_trans(:, obj.iterations + 1) = obj.dx_hat_L_trans + [0; 0; 0; 0; 0; obj.g];
            obj.dx_state_trans(:, obj.iterations + 1) = obj.dx_sys_trans;
            obj.dx_stateUDE_rot(:, obj.iterations + 1) = obj.dx_hat_UDE_rot;
            obj.dx_stateL_rot(:, obj.iterations + 1) = obj.dx_hat_L_rot;
            obj.dx_state_rot(:, obj.iterations + 1) = obj.dx_sys_rot;
            obj.time_array(obj.iterations + 1) = obj.iterations * obj.dt;
            obj = obj.updateDisturbanceArray(obj.disturbance_trans, obj.disturbance_rot);
        end

        function obj = setDisturbance(obj, disturbance_vector_trans, disturbance_vector_rot)
            obj.disturbance_trans = disturbance_vector_trans;
            obj.disturbance_rot = disturbance_vector_rot;
        end

        %% Functions for UDE observer (translational and rotational)

        function obj = calculateStateUDE_trans(obj)
            x_t = [obj.p; obj.dp];
            xi_dot_trans = -obj.Omega_UDE_trans * obj.xi_UDE_trans - (obj.Omega_UDE_trans^2 * (obj.B_pinv_trans * x_t) + obj.Omega_UDE_trans * (obj.B_pinv_trans * obj.A_trans * x_t)) - obj.Omega_UDE_trans * (obj.u_thrust - [0; 0; obj.g * obj.mass]);
            obj.xi_UDE_trans = obj.xi_UDE_trans + obj.dt * xi_dot_trans;
            obj.w_hat_UDE_trans = obj.xi_UDE_trans + obj.Omega_UDE_trans * obj.B_pinv_trans * x_t;
            obj.dx_hat_UDE_trans = obj.A_trans * x_t + obj.B_trans * obj.u_thrust + obj.B_trans * obj.w_hat_UDE_trans;
            obj.x_hat_UDE_trans = obj.x_hat_UDE_trans + obj.dt * obj.dx_hat_UDE_trans;
        end

        function obj = calculateStateUDE_rot(obj)
            controlVar_test = obj.J * obj.u_torque * 5;
            [q_vec_0, q_vec_1, q_vec_2, q_vec_3] = parts(obj.q);
            x_r = [q_vec_1; q_vec_2; q_vec_3; obj.omega];
            xi_dot_rot = -obj.Omega_UDE_rot * obj.xi_UDE_rot - (obj.Omega_UDE_rot^2 * (obj.B_pinv_rot * x_r) + obj.Omega_UDE_rot * (obj.B_pinv_rot * obj.A_rot * x_r)) - obj.Omega_UDE_rot * (controlVar_test);
            obj.xi_UDE_rot = obj.xi_UDE_rot + obj.dt * xi_dot_rot;
            obj.w_hat_UDE_rot = obj.xi_UDE_rot + obj.Omega_UDE_rot * obj.B_pinv_rot * x_r;
            obj.dx_hat_UDE_rot = obj.A_rot * x_r + obj.B_rot * controlVar_test + obj.B_rot * obj.w_hat_UDE_rot;
            obj.x_hat_UDE_rot = obj.x_hat_UDE_rot + obj.dt * obj.dx_hat_UDE_rot;
        end

        %% Functions for Luenberger observer (translational and rotational)

        function obj = calculateStateL_trans(obj)
            x_t = [obj.p; obj.dp];
            y = obj.C_trans * x_t;
            obj.dx_hat_L_trans = obj.A_trans * obj.x_hat_L_trans + obj.B_trans * (obj.u_thrust - [0; 0; obj.g * obj.mass]) + obj.L_trans * (y - obj.C_trans * obj.x_hat_L_trans);
            obj.x_hat_L_trans = obj.x_hat_L_trans + obj.dt * obj.dx_hat_L_trans;
            obj.w_hat_L_trans = x_t - obj.x_hat_L_trans;
        end

        function obj = calculateStateL_rot(obj)
            [q_vec_0, q_vec_1, q_vec_2, q_vec_3] = parts(obj.q);
            x_r = [q_vec_1; q_vec_2; q_vec_3; obj.omega];
            y_r = obj.C_rot * x_r;
            obj.dx_hat_L_rot = obj.A_rot * obj.x_hat_L_rot + obj.B_rot * obj.tau + obj.L_rot * (y_r - obj.C_rot * obj.x_hat_L_rot);
            obj.x_hat_L_rot = obj.x_hat_L_rot + obj.dt * obj.dx_hat_L_rot;
            obj.w_hat_L_rot = x_r - obj.x_hat_L_rot;
        end

        function obj = update(obj)
            obj = obj.applyControl();
            obj = obj.updateState();
            obj.iterations = obj.iterations + 1;
            obj = updateDroneDataExtention(obj);
        end
    end
end

