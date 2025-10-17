clear all
close all
clc

Ts = 1/10;               
car = Car(Ts);        
Vs = 120/3.6;           

[xs, us] = car.steady_state(Vs);  
sys = car.linearize(xs, us);      
[sys_lon, ~] = car.decompose(sys); 
sys_lon_d = c2d(sys_lon, Ts);     
[A, B, C, D] = ssdata(sys_lon_d);
B = -B;  % Equation (9) of the statement

% Cost matrices
Q = diag([20, 54.5]);   
R = 1.5;               


% Computation of the Minimal Robust Invariant Set Computation
K = -dlqr(A, B, Q, R); 
Acl = A + B*K;
uTs = us(2);
W = Polyhedron('lb', uTs-0.5, 'ub', uTs+0.5);  
W = -B*W;                                  

% Initialize visualization
epsi_fig = figure('Name', 'Minimal Robust Invariant Set Computation');
hold on; 
grid on; 
axis equal;

% Initialize set sequence at origin
F{1} = Polyhedron('lb', [0;0], 'ub', [0;0]);

% Setup computation parameters
i = 1;
end_value = 1e-2;  
colors = winter(100);  % Color scheme for visualization

while true
    F{i+1} = F{i} + Acl^(i)*W;
    F{i+1}.minHRep();  % Minimize representation for efficiency
    
    % Visualize current iteration
    plot(F{i+1}, 'Color', colors(i,:), 'Alpha', 0.3);
    title(sprintf('Iteration %d', i));
    
    if norm(Acl^i) < end_value
        fprintf('Convergence after %d iterations\n', i);
        break;
    end
    
    % Iteration management
    i = i + 1;
    if i > 100
        warning('Maximum iterations reached in invariant set computation');
        break;
    end
    drawnow;
end

% Store final minimal robust invariant set
Epsilon = F{end}; 
Epsilon.minHRep();

% Finalize visualization
xlabel('Position Error');
ylabel('Velocity Error');
title('Minimal Robust Invariant Set');
savefig(epsi_fig, 'Minimal.fig');

% Definition of the constraints that will allow us to compute the tightened
% constraints sets
x_safe_pos = 8;              
x_safe = [x_safe_pos; 0];    
distance_min = 6;                 

X = Polyhedron('A', [-1 0], 'b', -(distance_min - x_safe_pos));

u_min = -1;  % Maximum braking
u_max = 1;   % Maximum acceleration
U = Polyhedron('lb', u_min, 'ub', u_max);

X_tilde = X - Epsilon;       
X_tilde.minHRep();
KEpsilon = K * Epsilon;             
U_tilde = U - KEpsilon;      
U_tilde.minHRep();

% Computation of the Terminal Controller that ensures the stability of the
% MPC
% More conservative weights for terminal control
R_control = R*2;      % Double input penalty - smoother control
Q_control = Q/2;      % Half of tracking weights - more conservative

[Kt, Qf] = dlqr(A, B, Q_control, R_control);
Kt = -Kt;
Acl_t = A + B*Kt;  % Terminal closed-loop system

% Extract constraint matrices from polytopes
Fx = X_tilde.A; 
fx = X_tilde.b; 
Fu = U_tilde.A; 
fu = U_tilde.b; 
F = [Fx; Fu*Kt];     % Combined constraint matrix
f = [fx; fu];        % Combined constraint bounds

chi_f = Polyhedron('A', F, 'b', f); % Terminal invariant set (chi_f to match the course)

% Iteratively compute maximal invariant set
max_iter = 100;
j = 0;
while j < max_iter
    prev_set = chi_f;
    %pre_set = Polyhedron('A', [F; F*Acl_t], 'b', [f; f]);
    pre_set = Polyhedron(chi_f.A * Acl_t, chi_f.b);
    pre_set.minHRep();
    chi_f = intersect(chi_f, pre_set);
    chi_f.minHRep();
    
    if chi_f == prev_set
        fprintf('Convergence for terminal set after %d iterations\n', j);
        break;
    end
    
    j = j + 1;
    if j == max_iter
        warning('Maximum iterations (%d) reached in terminal set computation', max_iter);
    end
end

if chi_f.isEmptySet()
    error(['Terminal set is empty - adjust xsafe_pos or controller tuning.\n'...
          'Current xsafe_pos: %.2f'], xsafe_pos);
end

figure('Name', 'Terminal Invariant Set');
hold on;
plot(chi_f, 'alpha', 0.3, 'color', 'red', 'DisplayName', 'Terminal Set');
title('Terminal Invariant Set');
xlabel('Relative Position Error (m)');
ylabel('Relative Velocity Error (m/s)');
grid on;

% Saving step in the 'tube_mpc_variables.mat' file for the longitudinal MPC
save('tube_mpc_variables.mat', ...
    'K', 'Kt', 'Qf', ...                    % Controllers
    'Q', 'R', ...                           % Tracking weights
    'Q_control', 'R_control', ...           % Terminal weights
    'Epsilon', 'chi_f', ...                 % Sets
    'X_tilde', 'U_tilde', ...               % Constraints
    'distance_min', 'x_safe_pos' ...        % Safety parameters
    );