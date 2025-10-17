classdef MpcControl_lat < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this, not used)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE

            % Cost matrices
            Q = diag([4, 4]);%val 4 4 
            R = 5;     %val 5
            
            % State constraints
            F = [1  0;   
                -1  0;    
                 0  1;    
                 0 -1];   
            f = [3.5;             
                 0.5;              
                 5*pi/180;         
                 5*pi/180];       
            
            % Input constraints
            M = [1; -1];          
            m = [30*pi/180;        
                 30*pi/180];       
            
            % Terminal controller for stability
            [Kt, Qf, ~] = dlqr(mpc.A, mpc.B, Q, R);  % LQR solution
            Kt = -Kt;  
            % Compute maximal invariant set for terminal constraints
            Xf = polytope([F; M*Kt], [f; m]); 
            Acl = mpc.A + mpc.B*Kt;           
            while 1
                prevXf = Xf;
                [T, t] = double(Xf);
                preXf = polytope(T*Acl, t);    % One-step backward reachable set
                Xf = intersect(Xf, preXf);     % Intersect with current set
                if isequal(prevXf, Xf)
                    break
                end
            end
            [Ff, ff] = double(Xf);  % Get final polytope representation
            
            % Visualize constraint sets (optional but helpful for debugging)
            figure
            hold on; grid on;
            plot(polytope(F,f), 'g');  % State constraints in green
            plot(Xf, 'r');            % Terminal set in red
            xlabel('y position [m]');
            ylabel('Steering angle [rad]');
            
            % Set up optimization variables
            X = sdpvar(nx, N);     % State trajectory [y; theta]
            U = sdpvar(nu, N-1);   % Input trajectory [steering_angle]
            
            con = (X(:,1) == x0 - mpc.xs); 
            
            % Build constraints and objective over prediction horizon
            obj = 0;
            for k = 1:N-1
                % Dynamic constraints (deviation coordinates)
                con = con + (X(:,k+1) == mpc.A*X(:,k) + mpc.B*U(:,k));
                con = con + (F*(X(:,k) + mpc.xs) <= f);
                con = con + (M*(U(:,k) + mpc.us) <= m); 
                error_x = X(:,k) - (x_ref - mpc.xs);
                error_u = U(:,k) - (u_ref - mpc.us);
                obj = obj + error_x'*Q*error_x + error_u'*R*error_u;
            end
            
            con = con + (Ff*(X(:,N) + mpc.xs) <= ff);  
            error_x_final = X(:,N) - (x_ref - mpc.xs);
            obj = obj + error_x_final'*Qf*error_x_final;
            
            con = con + (u0 == U(:,1) + mpc.us);
            
            % Store debugging variables
            debugVars = {X, U};
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, x_ref, u_ref, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [xs_ref, us_ref] = compute_steady_state_target(mpc, ref)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Steady-state system
            A = mpc.A;
            B = mpc.B;

            % Linearization steady-state
            xs = mpc.xs;
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            xs_ref = [ref; 0];
            us_ref = 0;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
