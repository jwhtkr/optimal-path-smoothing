classdef RangeSensor < handle
    %RangeSensor Creates a sensor based on a limited range
    
    properties (SetAccess = protected, GetAccess = public)
        n_lines = 20; % Number of range measurements
        max_dist = 10; % Max distance of the range measurements
    end
    
    properties (SetAccess = protected, GetAccess = public)
        orien_nom % Stores the nominal orientation of each of the range lines
        
        % Obstacle plots
        h_obst_pos = []; % Handles to the obstacle posiiton plots
        h_obs_lines = [];% Handles to the ranging lines
    end
    
    properties (SetAccess = protected, GetAccess = public)
        ind_left = [] % indices for sensors on the left of the vehicle
        n_left = 0 % Number of left sensors
        ind_right = [] % indices for sensors on the right of the vehicle
        n_right = 0 % Number of right sensors
        ind_front = [] % indices for sensors pointing to the front of the vehicle
        n_front = 0 % Number of sensors pointing to the front of the vehicle
    end
    
    methods (Access = public)
        function obj = RangeSensor()
            % Create the nominal orientations of the range lines
            if obj.n_lines <= 1 % Single line sensor directly out front
                obj.orien_nom = 0;
            else
                obj.orien_nom = linspace(0,2*pi,obj.n_lines+1);
                obj.orien_nom = obj.orien_nom(1:end-1); % Remove the duplicate at 2pi
                
                % Adjust so that the first line is not straight forward
                delta = obj.orien_nom(2) - obj.orien_nom(1);
                obj.orien_nom = obj.orien_nom - delta/2;
            end
            
            % Initialize the sensor indices for left and right
            for k = 1:length(obj.orien_nom)
                % Get angle and adjust it to be between -pi and pi
                th = obj.orien_nom(k);
                th = atan2(sin(th), cos(th));
                
                % Add it to the left if orientation is positive
                if th > 0
                    obj.ind_left(end+1) = k;
                else % Otherwise consider it on the right of the vehicle
                    obj.ind_right(end+1) = k;
                end
                
                % Add it to the front if the angle is less than pi/2
                if abs(th) < (pi/2)
                    obj.ind_front(end+1) = k;                    
                end
            end
            
            % Set the count for the left and right fields
            obj.n_left = length(obj.ind_left);
            obj.n_right = length(obj.ind_right);
            obj.n_front = length(obj.ind_front);
        end
        
        function [xo, yo, dist_o] = getObstacleDetections(obj,q, th, world)
            %getDistanceToObstacles Summary of this method goes here
            %   q: position of the robot
            %   th: orientation of the robot
            %   world: instantiation of the PolygonWorld class
            %
            % Return values:
            %   xo: x indices of the obstacles
            %   yo: y indices of the obstacles
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Create the lines based on the current position and
            % orientation
            [x, y] = pol2cart(obj.orien_nom+th, obj.max_dist);
            x = x + q(1);
            y = y + q(2);
            
            % Initialize the obstacle points
            xo = ones(obj.n_lines, 1) * inf;
            yo = ones(obj.n_lines, 1) * inf;
            dist_o = ones(obj.n_lines, 1) * inf;
            
            % Loop through all of the polygons to get the location of the
            % obstacle points   
            for l = 1:obj.n_lines
                % Create the line formed by the range sensor
                line = [q, [x(l); y(l)]];
                
                % Loop through all of the polygons
                for k = 1:world.n_polygons
                    % Calcualte the distance to the polygon
                    [xk, yk, dist_tmp] = obj.getLineDetection(line, world.polygons{k});
                    
                    % Store data if the distance is less than previously
                    % seen
                    if dist_tmp < dist_o(l)
                        dist_o(l) = dist_tmp;
                        xo(l) = xk;
                        yo(l) = yk;
                    end
                end
            end
            
        end
        
        function initializePlots(obj, ax);
            % create the obstacle position plots
            obj.h_obst_pos = plot(ax, 0, 0, 'ro', 'linewidth', 2); hold on;
            set(obj.h_obst_pos, 'xdata', [], 'ydata', []);
            
            % Create the lines plot
            obj.h_obs_lines = cell(obj.n_lines, 1);
            for k = 1:obj.n_lines
                % Create the plot
                obj.h_obs_lines{k} = plot(ax, 0, 0, ':r', 'linewidth', 1);
            end
        end
        
        function plotMeasurements(obj, q, xo, yo)
            if isempty(obj.h_obst_pos)
                return;
            end
            
            % Plot the obstacle points
            set(obj.h_obst_pos, 'xdata', xo, 'ydata', yo);
            
            % Loop through and update the lines
            for k = 1:obj.n_lines
                xdata = [q(1) xo(k)];
                ydata = [q(2) yo(k)];
                set(obj.h_obs_lines{k}, 'xdata', xdata, 'ydata', ydata);
            end
        end
    end
    
    methods (Access = protected)
        function [xo, yo, dist] = getLineDetection(obj, line, polygon)
            % Calculates the position of the nearest obstacle detection
            % along the given line to the polygon
            %
            % Inputs:
            %   line: [x; y] - definition of a line - the first column is
            %   assumed to be the origin of the robot
            %   polygon: [x; y] - definition of a polygon through a series
            %   of lines
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Calculate the intersection points
            [xi, yi] = intersections(line(1,:), line(2,:), [polygon(1,:) polygon(1,1)], [polygon(2,:) polygon(2,1)]);
            
            % loop through the calculation points and determin the closest
            % obstacle
            if isempty(xi) || isempty(yi)
                xo = xi;
                yo = yi;
                dist = inf;
            end
            
            % Temporary debug plot
%             figure;
%             plot(polygon(1,:), polygon(2,:), 'r'); hold on;
%             plot(line(1,:), line(2,:), 'b');
%             plot(xi, yi, 'go');
            
            % Store the minimum distance
            q = line(:,1); % Position of the robot
            dist = inf;
            n_intersections = length(xi);
            for i = 1:n_intersections
                qi = [xi(i); yi(i)];
                
                % Calculate the distance to the obstacle
                dist_tmp = norm(q-qi);
                
                % Store the smallest distance found
                if dist_tmp < dist
                    dist = dist_tmp;
                    xo = xi(i);
                    yo = yi(i);
                end
            end
            
%             plot(xo, yo, 'ro', 'linewidth', 2);
        end
    end
end

