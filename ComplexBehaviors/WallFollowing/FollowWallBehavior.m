classdef FollowWallBehavior < handle
    %FollowWallBehavior Is a class which will provide a vector to follow a
    %given wall
    
    properties (SetAccess = protected)
        % Vector fields
        line_vf % Used to create a line
        avoid_field % Uses avoid on sensors
        total_field % Summation of line_vf and avoid_vf
        
        % Line variables
        vd % desired velocity
        ind_side % Indices of sensors to be used for initialization
        n_side % Number of side sensors used for wall detection
        
        % Avoidance variables
        n_avoid_sensors % number of sensors to be used (needed for the
                        % avoid vector field)
        ind_avoid % Indicies of the sensors to be used for obstacle avoidance
        
        % Line side identifications
        follow_left % true => follow line on left of vehicle, false => follow line on right of vehicle
        wall % Instance of ObstacleSideIdentification, used for calculating the wall
        dist_to_wall % Radius to maintain from obstacle points
        
        % Plotting variables
        h_line = [] % Used to plot the line denoting the wall
        h_wall = [] % Handle for plotting wall
        color = [] % Color used for plotting the vector field
        
    end
    
    properties(Constant)
        slope = 1; % Slope of sigmoid to the line        
    end
    
    methods
        function obj = FollowWallBehavior(follow_left, vd, dist_cont, ind_avoid, ind_side, dist_to_wall, color)
            %FollowWallBehavior Construct an instance of this class
            %   
            % Inputs:
            %   vd: desired velocity
            %   dist_cont: Distance for which an obstacle is considered
            %              continuous (used to calculate line to follow)
            %   n_avoid_sensors: 
            %   dist_to_wall: Radius to maintain from obstacle points
            
            % Store input variables
            obj.follow_left = follow_left;
            obj.vd = vd;
            obj.dist_to_wall = dist_to_wall;
            obj.ind_avoid = ind_avoid;
            obj.n_avoid_sensors = length(ind_avoid);
            obj.ind_side = ind_side;
            obj.n_side = length(ind_side);
            obj.color = color;
            
            % Initialize the wall variables
            obj.wall = ObstacleSideIdentification(dist_cont);
            
            % Plotting variables (used to visualize the vector field)
            x_vec = -1:1:20;
            y_vec = -6:1:10;
            
            % Calculate vector field parameters
            S_avoid = 0.9 * dist_to_wall; % Turn off influence close to the desired distance
            R_avoid = 0.5 * dist_to_wall; % Turn on avoidance radius halfway to the wall
            q_inf = [10000000; 10000000];
            
            % Initialize line vector field
            obj.line_vf = LineVectorField(x_vec, y_vec, [0;0], 0, obj.slope, obj.vd);
            
            % Create the obstacle avoid vector fields
            fields_avoid = cell(obj.n_avoid_sensors, 1); % Objective avoid field
            for k = 1:obj.n_avoid_sensors
                %fields_avoid{k} = AvoidObstacleConst(x_vec, y_vec, obj.q_inf, obj.g_max, obj.S);
                fields_avoid{k} = AvoidObstacle(x_vec, y_vec, q_inf, obj.vd);
                fields_avoid{k}.S = S_avoid;
                fields_avoid{k}.R = R_avoid;
            end
            w_avoid = 1;
            weights_avoid = w_avoid*ones(obj.n_avoid_sensors, 1);
            obj.avoid_field = SummedFields(x_vec, y_vec, fields_avoid, weights_avoid, obj.vd);
            
            % Store the line vector field and avoidance vector field in one
            % summed field
            obj.total_field = SummedFields(x_vec, y_vec, {obj.avoid_field, obj.line_vf}, [1;1], obj.vd);            
        end
        
        function updateVectorFields(obj, q_sensors, ind_inf, q_veh, th_veh)
            %updateVectorFields Updates the vector field used in this
            %behavior
            %
            % Inputs:
            %   q_sensors: 2xm matrix of sensor measurements
            %   ind_inf: 1xm indicies of measurements that are invalid
            %            where ind_inf(k) indicates that a measurement was
            %            not recieved for sensor k. q_sensors(:,k) would
            %            then be ignored
            %    q_veh: 2x1 position vector of the vehicle
            %    th_veh: scalar orientation of the vehicle
            
            % Set avoidance vector fields
            for k = 1:obj.n_avoid_sensors
                ind = obj.ind_avoid(k);
                obj.avoid_field.fields{k}.x_o = q_sensors(:,ind);
            end
            
            % Initialize the wall if needed
            if ~obj.wall.initialized
            
                % Get the obstacles corresponding to the side to be
                % initialized
                q_side = [];
                n_side_used = 0;
                for k = 1:obj.n_side
                    ind = obj.ind_side(k);
                    if isempty(find(ind_inf == ind))
                        n_side_used = n_side_used + 1;
                        q_side(:, n_side_used) = q_sensors(:, ind);
                    end
                end
                
                % Initialize the left side
                obj.wall.initializePointOnWall(q_side, q_veh, th_veh);
            end
            
            % Find the contiguous portion of the wall
            [q_wall, ~] = obj.wall.findContinguousWall(q_sensors, q_veh);
            obj.h_wall = obj.plotWall(q_wall, obj.h_wall);            
            
            % Calculate the line to follow
            if size(q_wall, 2) > 1
               % Calculate line using first and last point
                q0 = q_wall(:, 1);
                vl = q_wall(:, end) - q0;
                vl = vl ./norm(vl);

                % Adjust v to point in same direction as vehicle
                orien_vec = [cos(th_veh) sin(th_veh)]; % Orientation vector
                if orien_vec*vl < 0 % The line is pointing in the wrong direction
                    vl = -vl;
                    q0 = q_wall(:, end);
                end

                % Calculate offset orthogonal vector. If follow left
                % then v needs to be rotated by pi/2 if follow right
                % then -pi/2
                R = [0 -1; 1 0]; % right rotation
                if obj.follow_left 
                    R = -R; % left rotation
                end
                v_orth = R*vl;

                % Calculate a point on the line to be followed
                ql = q0 + v_orth*obj.dist_to_wall;

                % Set the line parameters
                obj.line_vf.setLineParameters(ql, atan2(vl(2), vl(1)));

                if isempty(obj.h_line)
                    % Plot the line
                    q1 = q0 + vl*5;
                    obj.h_line = plot([q0(1) q1(1)], [q0(2) q1(2)], 'k', 'linewidth', 2);
                else
                    % Plot the line
                    q1 = q0 + vl*5;
                    set(obj.h_line, 'xdata', [q0(1) q1(1)], 'ydata', [q0(2) q1(2)]);                        
                end
            else
                warning('Insufficient data points for line calculation');
            end
            
        end
        
        function g = getVector(obj, t, q, th)
            %getVector Returns the vector to follow the line
            % 
            % Inputs:
            %   t: time index
            %   q: 2x1 position vector
            %   th: scalar orientation variable
            g = obj.total_field.getVector(t, q, th);
        end
    end
    
     methods
        function h_plot = plotWall(obj, q_wall, h_plot)
            xdata = q_wall(1, :);
            ydata = q_wall(2, :);
            if isempty(h_plot)
                h_plot = plot(xdata, ydata, 'o', 'color', obj.color, 'linewidth', 3);                
            else
                set(h_plot, 'xdata', xdata, 'ydata', ydata);
            end
        end
    end
end

