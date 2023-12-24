classdef MpcControl_x < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            deltaX = X - x_ref;
            deltaU = U - u_ref;

            f = [deg2rad(10); deg2rad(10)];
            m = [0.26, 0.26]';
            
            F = [0 1 0 0;0 -1 0 0];
            M = [1 -1]';

            Q = eye(nx);
            R = eye(nu);
            [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
            K = -K;

            Xf = polytope([F;M*K],[f;m]);
            Acl = mpc.A+mpc.B*K;
            while 1
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf, preXf);
                if isequal(prevXf, Xf)
                    break
                end
            end
            Xf = Xf + xs;
            figure
            Xf.projection(1:2).plot();
            xlabel('\omega_y')
            ylabel('\beta')
            figure
            Xf.projection(2:3).plot();
            xlabel("\beta")
            ylabel("v_x")
            figure
            Xf.projection(3:4).plot();
            xlabel("v_x")
            ylabel("x")
            [Ff,ff] = double(Xf);
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = deltaU(:,1)'*R*deltaU(:,1);
            con = (deltaX(:,2) == mpc.A*deltaX(:,1) + mpc.B*deltaU(:,1)) + (M*deltaU(:,1) <= m) +(F*deltaX(:,1) <= f);
            for k = 2:N-1
                con = [con, deltaX(:,k+1) == deltaX(:, 1)];
                con = [con, deltaX(:,k+1) == mpc.A * deltaX(:,k) + mpc.B * deltaU(:,k)];
                con = [con,(F*deltaX(:,k) <= f - F*x_ref) + (M*deltaU(:,k) <= m - M*u_ref)];
                obj = obj + deltaX(:,k)'*Q*deltaX(:,k) + deltaU(:,k)'*R*deltaU(:,k);
            end
            obj = obj + deltaX(:,N)'*Qf*deltaX(:,N);
            con = [con, Ff*deltaX(:,N) <= ff];
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            R = eye(nx);

            f = [deg2rad(10); deg2rad(10)];
            m = [0.26, 0.26]';
            
            F = [0 1 0 0;0 -1 0 0];
            M = [1 -1]';

            obj = us'*R*us;
            con = xs == eye(size(mpc.A)) - mpc.A * xs - mpc.B * us;
            con = [con, ref == mpc.C*us];
            con = [con,(F*xs <= f) + (M*us <= m)];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
