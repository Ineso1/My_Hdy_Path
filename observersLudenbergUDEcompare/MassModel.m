classdef MassModel
    properties
        mass
        b
        kp
        kd
        disturbance
        observerNumber

        A
        A_L
        B
        C
        D
        B_pinv
        u 
        e

        dt
        t
        n

        x
        x_hat   

        dx 
        dx_hat 

        desired_eigenvalues
        L     
        alpha_L
        beta_L
        Omega   
        xi
        w_hat
        dw_hat
        gamma  
        kd_smc  
        lambda
        dlambda 
        disturbance_bound  
        lambda_0   
        lambda_1   
        eta   

        x_arr
        dx_arr
        x_hat_arr
        dx_hat_arr
        w_hat_arr
        u_arr
        e_arr

        feedback

        iterations
    end    

    methods
        function obj = MassModel(mass, b, dt, t)
            obj.mass = mass;
            obj.b = b;
            obj.dt = dt;
            obj.t = t;
            obj.n = length(t);
        
            obj.A = [0 1; 0 -b/mass];
            obj.A_L = [0 1 0 0; 0 -b/mass 1/mass 0; 0 0 0 1; 0 0 -1 0];
            obj.B = [0; 1/mass];      
            obj.C = [1 0];          
            obj.D = 0;
        
            obj.B_pinv = pinv(obj.B);  
        
            obj.u = 0;
            obj.e = 0;
            obj.x = zeros(2, 1);
            obj.dx = zeros(2, 1);
            obj.x_hat = zeros(2, 1);
            obj.dx_hat = zeros(2, 1);
        
            obj.x_arr = [];
            obj.dx_arr = [];
            obj.x_hat_arr = [];
            obj.dx_hat_arr = [];
            obj.w_hat_arr = [];
            obj.u_arr = [];
            obj.e_arr = [];

            obj.desired_eigenvalues = [-5, -8, -10, -12];
            obj.L = place(obj.A_L', [obj.C, 0, 0]', obj.desired_eigenvalues)';
            obj.alpha_L = 0;
            obj.beta_L = 0;

            obj.Omega = 10;
            obj.xi = -obj.Omega * (obj.B_pinv * obj.x);
            obj.w_hat = 0;

            obj.gamma = 0.1;
            obj.kd_smc = 10.5;

            obj.lambda = 0.001;
            obj.dlambda = 1.0;
            obj.disturbance_bound = 15;
            obj.lambda_0 = 1.5 * sqrt(obj.disturbance_bound);
            obj.lambda_1 = 1.1 * obj.disturbance_bound;
            obj.eta = 0;

            obj.iterations = 0;

            obj.feedback = 0;
        end

        function obj = setControlGains(obj, kp, kd)
            obj.kp = kp;
            obj.kd = kd;
        end

        function obj = setDisturbance(obj, disturbance)
            obj.disturbance = disturbance;
        end

        function obj = setUDEGain(obj, gain)
            obj.Omega = gain;
            obj.xi = -obj.Omega * (obj.B_pinv * obj.x);
        end

        function obj = setLudenbergerEign(obj, desired_eigenvalues)
            obj.desired_eigenvalues = desired_eigenvalues;
            obj.L = place(obj.A_L', [obj.C, 0, 0]', obj.desired_eigenvalues)';
        end

        function obj = setSMCDiffGains(obj, gamma, kd_smc)
            obj.gamma = gamma;
            obj.kd_smc =  kd_smc;
        end

        function obj = setSTGains(obj, lambda, dlambda, k_lambda_0, k_lambda_1, disturbance_bound)
            obj.lambda = lambda;
            obj.dlambda = dlambda;
            obj.disturbance_bound = disturbance_bound;
            obj.lambda_0 = k_lambda_0 * sqrt(obj.disturbance_bound);
            obj.lambda_1 = k_lambda_1 * obj.disturbance_bound;
        end

        function obj = selectObserver(obj, number)
            if number >= 1 && number <= 5
                obj.observerNumber = number;
                if number == 1 && obj.iterations == 0
                    obj.A = obj.A_L;
                    obj.B = [obj.B; 0; 0];
                    obj.C = [obj.C, 0, 0];
                    obj.x = zeros(4, 1);
                    obj.dx = zeros(4, 1);
                    obj.x_hat = zeros(4, 1);
                    obj.dx_hat = zeros(4, 1);
                end
            else
                obj.observerNumber = 0; 
            end
        end

        function obj = update(obj, ref)
            obj = obj.stateObserver();
            
            if obj.feedback > 0
                obj.e = ref - obj.x(1); 
                de = -obj.dx_hat(1);

                if obj.observerNumber == 1
                    de = -obj.dx_hat(1);
                    obj.u = obj.kp * obj.e + obj.kd * de - obj.w_hat ;
                elseif obj.observerNumber == 2 
                    obj.u = obj.kp * obj.e + obj.kd * de - obj.w_hat;
                elseif obj.observerNumber == 4 
                    de = -obj.dx(1);
                    obj.u = obj.kp * obj.e + obj.kd * de - obj.w_hat;
                elseif obj.observerNumber == 5
                    de = -obj.dx(1);
                    obj.u = obj.kp * obj.e + obj.kd * de - obj.w_hat;
                else
                    obj.u = obj.kp * obj.e + obj.kd * de;
                end
            else
                obj.e = ref - obj.x(1); 
                de = -obj.x(2);
                obj.u = obj.kp * obj.e + obj.kd * de;  
            end

            F_disturbance = obj.disturbance;

            obj.dx = obj.A * obj.x + obj.B * (obj.u + F_disturbance);
            obj.x = obj.x + obj.dt * obj.dx; 

            obj = obj.updateArrs();
            obj.iterations = obj.iterations + 1;
        end

        function obj = stateObserver(obj)
            switch obj.observerNumber
                case 1
                    % Luenberger Observer
                    y = obj.C * obj.x;
                    obj.dx_hat = obj.A_L * obj.x_hat + obj.B * obj.u + obj.L * (y - obj.C * obj.x_hat);
                    obj.x_hat = obj.x_hat + obj.dt * obj.dx_hat;
                    obj.w_hat = obj.x_hat(3);
                    obj.dw_hat = obj.x_hat(4);

                case 2
                    % Uncertainty and Disturbance Estimator (UDE) Observer
                    xi_dot = -obj.Omega * obj.xi - (obj.Omega * obj.Omega * (obj.B_pinv * obj.x)) - obj.Omega * obj.u;
                    obj.xi = obj.xi + obj.dt * xi_dot;
                    obj.w_hat = obj.xi + obj.Omega * (obj.B_pinv * obj.x);
                    obj.dx_hat = obj.A * obj.x + obj.B * obj.u + obj.B * obj.w_hat;
                    obj.x_hat = obj.x_hat + obj.dt * obj.dx_hat;

                case 3
                    % Sliding Mode Observer Differentiator (SMO)
                    e_smc = obj.x_hat(1) - obj.x(1);
                    e_smc_vel = obj.x_hat(2) - obj.x(2);
                    v_injection = -obj.gamma * sign(e_smc) - obj.kd_smc * e_smc_vel;
                    obj.dx_hat(1) = obj.x_hat(2);     
                    obj.dx_hat(2) = v_injection;      
                    obj.x_hat = obj.x_hat + obj.dt * obj.dx_hat;

                case 4
                    % Super-Twisting Observer
                    e_ST = obj.e;  
                    de_ST = - obj.x(2);  
                    sigma = obj.dlambda * de_ST + obj.lambda * e_ST;  
                    s = sigma + obj.x_hat(2);  
                    v_injection_ST = -obj.lambda_0 * sign(s) * sqrt(abs(s)) - obj.eta;
                    obj.eta = obj.eta + obj.dt * obj.lambda_1 * sign(s);
                    obj.dx_hat(1) = obj.x_hat(2);     
                    obj.dx_hat(2) = v_injection_ST;   
                    obj.x_hat = obj.x_hat + obj.dt * obj.dx_hat;
                    obj.w_hat = v_injection_ST; 
                    obj.w_hat = (obj.dx(2) - obj.dx_hat(2));

                case 5
                    % Super-Twisting Observer (STO)
                    e1 = obj.x(1) - obj.x_hat(1);
                    z1 = 1 * abs(e1)^(1/2) * sign(e1);
                    z2 = 0.01 * sign(e1);
                    obj.dx_hat(1) = obj.x_hat(2) + z1;
                    v_injection = (obj.A * obj.x_hat + obj.B * obj.u);
                    obj.dx_hat(2) = v_injection(2) + z2;
                    obj.x_hat = obj.x_hat + obj.dt * obj.dx_hat;
                    obj.w_hat = (obj.dx(2) - obj.dx_hat(2));

                otherwise
                    % No Observer
                    obj.x_hat = obj.x;
                    obj.dx_hat = obj.dx;
            end
        end

        function obj = updateArrs(obj)
            obj.x_arr(:, obj.iterations + 1) = obj.x;
            obj.dx_arr(:, obj.iterations + 1) = obj.dx;
            obj.x_hat_arr(:, obj.iterations + 1) = obj.x_hat;
            obj.dx_hat_arr(:, obj.iterations + 1) = obj.dx_hat;
            obj.u_arr(:, obj.iterations + 1) = obj.u;
            obj.e_arr(:, obj.iterations + 1) = obj.e;
            obj.w_hat_arr(:, obj.iterations + 1) = obj.w_hat;
        end
    end
end
