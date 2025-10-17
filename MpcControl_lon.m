classdef MpcControl_lon < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   V_ref, u_ref - reference state/input
            %   d_est        - disturbance estimate
            %   x0other      - initial state of other car
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            V_ref = sdpvar(1);
            u_ref = sdpvar(1);

            % Disturbance estimate (Ignore this before Todo 4.1)
            d_est = sdpvar(1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this before Todo 5.1)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system.
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
          
            % Load step from the 'tube_mpc_variables.mat' file
            variables = load('tube_mpc_variables.mat');
            xsafe = [variables.x_safe_pos; 0];  
            
            % Cost matrices
            Q = variables.Q;    % State cost matrix
            R = variables.R;    % Input cost matrix

            Fx_const = variables.X_tilde.A;     
            fx_const = variables.X_tilde.b;     
            Fu_const = variables.U_tilde.A;  
            fu_const = variables.U_tilde.b;    

            % Setup optimization variables
            X = sdpvar(nx, N);    
            U = sdpvar(nu, N-1);   
            
            % Initialize optimization problem
            obj = 0;   
            con = [];  
            con = con + (X(:,1) == x0other - x0 - xsafe);
            
            % Build constraints and objective over prediction horizon
            for k = 1:N-1
                con = con + (X(:,k+1) == mpc.A*X(:,k) - mpc.B*U(:,k)); % Eq(9) from statement
                con = con + (Fx_const*X(:,k) <= fx_const);
                con = con + (Fu_const*U(:,k) <= fu_const);  
                obj = obj + X(:,k)'*Q*X(:,k) + U(:,k)'*R*U(:,k);
            end
            obj = obj + X(:,N)'*variables.Qf*X(:,N);
            
            F_term = variables.chi_f.A;
            f_term = variables.chi_f.b;
            con = con + (F_term*X(:,N) <= f_term); % Terminal set constraints
            
            state_error = x0other - x0 - xsafe - X(:,1); 
            con = con + (u0 == U(:,1) + variables.K*state_error);% Tube controller
            
            % Store variables for debugging
            debugVars = {X, U};
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0, debugVars{:}});
        end
        
 function [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate (Ignore before Todo 4.1)
            % OUTPUTS
            %   Vs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Steady-state subsystem
            A = mpc.A(2, 2);
            B = mpc.B(2, 1);

            % Subsystem linearization steady-state
            xs = mpc.xs(2);
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            Vs_ref = ref;
            us_ref = 0;

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end