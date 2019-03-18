classdef VehicleKinematics < handle
    %VehicleKinematics Abstract class defining the required methods for a dynamics
    % class    
    
    properties (GetAccess = public, SetAccess = public)
        % State variables
        x_ind = 1   % Index into state of x position
        y_ind = 2   % Index into state of y position
        th_ind = 3  % Index into state of orientation
        dimensions = 3 % Number of dimensions of state
        
        % Plotting vehicle variables
        h_veh = [] % Handle of the agent
        fig = [] % Handle to the figure
        ax = [] % Handle to the axis
        r = 0.2 % radius parameter for plotting the size of the agent
        c = 'b' % color of the agent
        
        % Plotting path variables
        h_path = [] % Handle to the path of the agent
        path_period = 0.1 % Plot the path every path_period seconds
        next_path_time = -1 % Replot the path again once time > next_path_time
        plot_path = false % true => plot the path the the robot has traveled
        
    end
    
    methods (Abstract)
        xdot = kinematics(obj, t, x, u); % dynamics given current time(t), state (x), and input(u) 
        [v, w] = getVelocities(obj, t, x, u);
    end
    
    methods
        function obj = VehicleKinematics(dimensions)
            obj.dimensions = dimensions;
        end
        
        function initializeStatePlot(obj, ax, x)
            % Get the robot polygon
            P = obj.getRobotPolygon(x);
            
            % Plot the triangle
            obj.h_veh = fill(ax, P(:, 1), P(:, 2), obj.c, 'Edgecolor', obj.c);
            hold on; 
            axis equal;
            
            if obj.plot_path
                % Set the figure to be the same as the vehicle was
                % plotted in
                
                % Plot the initial path point
                obj.h_path = plot(ax, q(1), q(2), [':' obj.c], 'linewidth', 3);                    

                % Set the time for the next plotting
                obj.next_path_time = t + obj.path_period;
            end            
        end
        
        function plotState(obj, t, x)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % This function plots the state of the robot.  Uses
            % a triangular plot to plot position and orientation
            %
            % t: time of plot
            % x: state of the vehicle            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if isempty(obj.h_veh)
                return;
            end             
            
            % Get the robot polygon
            P = obj.getRobotPolygon(x);            

            % Plot the triangle
            set(obj.h_veh, 'xdata', P(:, 1), 'ydata', P(:, 2));                
                        
            % Plot the path
            if obj.plot_path
               if t >= obj.next_path_time
                    % Form the aggregate path
                    x_vec = [get(obj.h_path, 'xdata') q(1)];
                    y_vec = [get(obj.h_path, 'ydata') q(2)];

                    % Replot
                    set(obj.h_path, 'xdata', x_vec, 'ydata', y_vec);

                    % Reset the time
                    obj.next_path_time = t + obj.path_period;
               end                
            end
            
        end
    
        function P = getRobotPolygon(obj, x)
            % Rotation matrix (for finding points on triangle)
            R = [cos(x(obj.th_ind)) -sin(x(obj.th_ind)); sin(x(obj.th_ind)) cos(x(obj.th_ind))];

            % Points on triangle (descriptions are for pi/2 rotation)
            q = [x(obj.x_ind); x(obj.y_ind)];
            p1 = R * [-obj.r; -obj.r] + q;      % right corner of triangle
            p2 = R * [0; 0] + q;       % tip of triangle
            p3 = R * [-obj.r; obj.r] + q;       % Left corner of triangle
            p4a = R * [-obj.r; 0] + q;
            p4 = (q + p4a) ./ 2;            % mid point of triangle
            P = [p1'; p2'; p3'; p4'];
        end
        
        function plotVelocitiesAndInput(obj, tmat, xmat, u)
            % Create velocity vectors
            n = length(tmat);
            v_vec = zeros(1, n);
            w_vec = zeros(1,n);
            u_vec = zeros(2,n);
            for k = 1:n
                t = tmat(k);
                x = xmat(:,k);
                u_vec(:,k) = u(t, x);
                [v_vec(k), w_vec(k)] = obj.getVelocities(t, x, u_vec(:,k)); 
            end
            
            % Create a new figure
            figure
            
            % Plot Translational velocities
            subplot(3,1,1);
            plot(tmat, v_vec, 'linewidth', 2);
            ylabel('v(t)')
            
            % Plot Rotational velocities
            subplot(3,1,2);
            plot(tmat, w_vec, 'linewidth', 2);
            ylabel('\omega(t)');
            
            % Plot Inputs
            subplot(3,1,3);
            plot(tmat, u_vec(1,:), 'linewidth', 2); hold on;
            plot(tmat, u_vec(2,:), 'linewidth', 2);
            ylabel('Inputs');
            xlabel('time (s)');
        end
    end
    
    
end

