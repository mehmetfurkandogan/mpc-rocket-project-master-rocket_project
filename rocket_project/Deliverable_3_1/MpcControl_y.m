classdef MpcControl_y < MpcControlBase
    
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
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            f = [deg2rad(10); deg2rad(10)];
            m = [0.26, 0.26]';
            
            F = [0 1 0 0;0 -1 0 0];
            M = [1 -1]';
            Q = diag([10, 1, 1, 50]);
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

            % Plotting the terminal invariant set
            f1 = figure('name','Terminal Invariant Set','numberTitle','off');
            Xf.projection(1:2).plot();
            xlabel('\omega_x (rad/s)')
            ylabel('\alpha (rad)')
            exportgraphics(f1,'plots/tis_y1.eps', BackgroundColor='none',ContentType='vector')

            f2 = figure('name','Terminal Invariant Set','numberTitle','off');
            Xf.projection(2:3).plot();
            xlabel("\alpha (rad)")
            ylabel("v_y (m/s)")
            exportgraphics(f2,'plots/tis_y2.eps', BackgroundColor='none',ContentType='vector')

            f3 = figure('name','Terminal Invariant Set','numberTitle','off');
            Xf.projection(3:4).plot();
            xlabel("v_y (m/s)")
            ylabel("y (m)")
            exportgraphics(f3,'plots/tis_y3.eps', BackgroundColor='none',ContentType='vector')

            [Ff,ff] = double(Xf);
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = U(:,1)'*R*U(:,1) + X(:,1)'*Q*X(:,1);
            con = (X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1)) + (M*U(:,1) <= m) +(F*X(:,1) <= f);
            for k = 2:N-1
                con = [con, X(:,k+1) == mpc.A * X(:,k) + mpc.B * U(:,k)];
                con = [con,(F*X(:,k) <= f) + (M*U(:,k) <= m)];
                obj = obj + X(:,k)'*Q*X(:,k) + U(:,k)'*R*U(:,k);
            end
            obj = obj + X(:,N)'*Qf*X(:,N);
            con = [con, Ff*X(:,N) <= ff];
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
            obj = 0;
            con = [xs == 0, us == 0];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
