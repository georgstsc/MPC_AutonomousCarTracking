function [velocity, throttle, dist, eps, chi] = plot_display(result, varargin)
% The plot_display function allows us to generate the different plots
% required for the report
%
% Inputs:
%   result : Structure containing simulation results
%   varargin : Optional input for save directory (default: 'plots')
%
% Outputs:
%   velocity : plot of the velocity along time 
%   throttle : plot of the throttle input along time
%   disturb : plot of the disturbance estimation along time
%   dist : plot of the relative distance between the cars along time
%   eps : plot of the minimal invariant set 
%   chi : plot of the terminal invariant set

    if nargin > 1
        save_dir = varargin{1};
    else
        save_dir = 'plots';
    end

    % We fetch the informations needed for the figures from the 'result'
    % calculation
    t = result.T;
    x1 = result.myCar.X(1,:);  % x position
    y1 = result.myCar.X(2,:);  % y position
    theta1 = result.myCar.X(3,:);  % heading
    v1 = result.myCar.X(4,:) * 3.6;  % velocity (converted to km/h)
    
    delta1 = rad2deg(result.myCar.U(1,:));  % steering angle in degrees
    throttle1 = result.myCar.U(2,:);  % throttle input
    
    % Figure of the velocity
    % We partially used the code provided in visualization.m
    velocity = figure('Name', 'Velocity', 'Position', [100, 100, 800, 400]);
    grid on;
    box on;
    hold on;
    plot(t, v1, 'b', 'LineWidth', 1.5, 'DisplayName', 'Car 1');
    if isfield(result.myCar, 'Ref')
        refv1 = result.myCar.Ref(2,:) * 3.6;
        stairs(t, refv1, '--b', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    end
    if isfield(result, 'otherCar')
        v2 = result.otherCar.X(4,:) * 3.6;
        plot(t, v2, 'r', 'LineWidth', 1.5, 'DisplayName', 'Car 2');
    end
    xlabel('Time [s]');
    ylabel('V [km/h]');
    title('Velocity');
    legend('show');
    saveas(gcf, fullfile(save_dir, 'velocity5.1.png'));
    
    % Figure of the throttle input
    % We partially used the code provided in visualization.m
    throttle = figure('Name', 'Throttle', 'Position', [200, 200, 800, 400]);
    grid on;
    box on;
    hold on;
    stairs(t, throttle1, 'b', 'LineWidth', 1.5, 'DisplayName', 'Car 1');
    if isfield(result, 'otherCar')
        throttle2 = result.otherCar.U(2,:);
        stairs(t, throttle2, 'r', 'LineWidth', 1.5, 'DisplayName', 'Car 2');
    end
    xlabel('Time [s]');
    ylabel('u_T');
    title('Throttle');
    legend('show');
    saveas(gcf, fullfile(save_dir, 'throttle5.1.png'));
    
    % Figure of the relative distance between the two cars
    % We partially used the code provided in visualization.m
    x2 = result.otherCar.X(1,:);
    dist = figure('Name', 'Relative Distance', 'Position', [350, 350, 800, 400]);
    grid on;
    box on;
    hold on;
    plot(t, x2-x1, 'b', 'LineWidth', 1.5, 'DisplayName', 'relative distance');
    xlabel('Time [s]');
    ylabel('dist [m]');
    title('Relative Distance to Car 2');
    legend('show');
    saveas(gcf, fullfile(save_dir, 'relative_distance5.1.png'));
    
    % Plots of the terminal invariant and minimal robust invariant sets
    variables = load('tube_mpc_variables.mat');
    eps = openfig('Minimal.fig');
    chi_f = variables.chi_f;  

    chi = figure('Name', 'Terminal Invariant Set');
    hold on;
    plot(chi_f, 'alpha', 0.3, 'color', 'red', 'DisplayName', 'Terminal Set');
    title('Terminal Invariant Set');
    xlabel('Relative Position Error (m)');
    ylabel('Relative Velocity Error (m/s)');
    grid on;
    legend('show');
    saveas(gcf, fullfile(save_dir, 'Terminal_set5.1.png'));

    

end