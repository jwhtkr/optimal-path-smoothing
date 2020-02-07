classdef SingleAgentPlotter < Plotter
    %SingleAgentPlotter plots a single agent
    
    properties
        color % 1x3 color vector
        r = 0.2 % radius parameter for plotting the size of the agent
    end
    
    properties(Access=protected)
        h_veh = [] % Handle of the agent
        getConfiguration % Handle to a function that will return the state given a time index        
        q_ind = 1:2; % Indices for the position
        th_ind = 3; % Indices for the orientation
    end
    
    methods
        function obj = SingleAgentPlotter(getConfiguration, color)
            obj = obj@Plotter();
            obj.getConfiguration = getConfiguration;
            obj.color = color;
        end
        
        %%%%%%  Plotting functions %%%%%%%%%%%
        function initializePlot(obj, t)
            % Get the robot polygon
            x = obj.getConfiguration(t);
            P = obj.getRobotPolygon(x);
            
            % Plot the triangle
            obj.h_veh = fill(P(:, 1), P(:, 2), obj.color, 'Edgecolor', obj.color);
            hold on; 
            axis equal;
        end
        
        function plot(obj, t)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % This function plots the state of the robot.  Uses
            % a triangular plot to plot position and orientation
            %
            % t: time of plot
            if isempty(obj.h_veh)
                return;
            end 
            
            % Get the robot polygon
            x = obj.getConfiguration(t);
            P = obj.getRobotPolygon(x);
            
            % Plot the triangle
            set(obj.h_veh, 'xdata', P(:, 1), 'ydata', P(:, 2));
        end
        
        function P = getRobotPolygon(obj, x)
            % Rotation matrix (for finding points on triangle)
            R = [cos(x(obj.th_ind)) -sin(x(obj.th_ind)); sin(x(obj.th_ind)) cos(x(obj.th_ind))];

            % Points on triangle (descriptions are for pi/2 rotation)
            q = x(obj.q_ind);
            p1 = R * [-obj.r; -obj.r] + q;      % right corner of triangle
            p2 = R * [0; 0] + q;       % tip of triangle
            p3 = R * [-obj.r; obj.r] + q;       % Left corner of triangle
            p4a = R * [-obj.r; 0] + q;
            p4 = (q + p4a) ./ 2;            % mid point of triangle
            P = [p1'; p2'; p3'; p4'];
        end
    end
end

