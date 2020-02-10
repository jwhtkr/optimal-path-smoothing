classdef WallFollowAgent < SingleAgent
    %WallFollowAgent Creates an agent that will follow a wall
    
    properties
        % Vector field for line following
        line_vf % Used to create a line
        follow_left % true => follow line on left of vehicle, false => follow line on right of vehicle
        
        avoid_field % Uses avoid on sensors
        total_field % Summation of line_vf and avoid_vf
        
        % sensor indices to use
        ind_left = [];
        n_left
        
        % Previous line
        line_detected = false;
        line_lost = false;
        q0 = [] % point on line
        vl = [] % vector pointing parallel to line
        v_orth = [] % vector orthogonal to line (pointing in direction where vehicle should be)
        h_line = []
        h_sensors = [];
        
        
        q_inf
        
        % Line side identifications
        wall_left % Instance of ObstacleSideIdentification, used for left side of vehicle
        h_left = [] % Hande for plotting left wall
        wall_right % Instance of ObstacleSideIdentification for right side of vehicle
        h_right = [] % Handle for plotting right wall
        dist_cont = 0.3;
    end
    
    properties (Constant)
        % Line variables
        vd = 1; % Desired velocity
        slope = 1; % Slope of sigmoid to the line
        offset = 0.5; % Offset from wall to follow
        gap_thresh = 20; % Threshold the distance to gap
        
        
        S_avoid = .4; % Sphere of influence of avoid
        R_avoid = 0.25; % Radius of full avoid
        
        % Sensing variables
        n_sensors = 30; % Number of sensor lines
        max_sensor_range = 4; % Maximum range of the sensor
    end
    
    
    methods
        function obj = WallFollowAgent(veh, world, follow_left)
            % initialize the scenario
            obj = obj@SingleAgent(veh, world); 
            obj.follow_left = follow_left;
            
            % Initialize sensors
            obj.vehicle.sensor.initializeSensor(obj.n_sensors, obj.max_sensor_range);
            
            % Initialize wall variables
            obj.wall_left = ObstacleSideIdentification(obj.dist_cont);
            obj.wall_right = ObstacleSideIdentification(obj.dist_cont);
            
            % Plotting variables
            x_vec = -1:1:20;
            y_vec = -6:1:10;
            
            % Initialize vector field
            obj.line_vf = LineVectorField(x_vec, y_vec, [0;0], 0, obj.slope, obj.vd);
            
            % Get the middle three sensors on the left to use for wall
            % following
            mid = round(veh.sensor.n_left/2);
            %obj.ind_left = (mid+2):-1:1; % back two to front
            obj.ind_left = (mid+1):-1:(mid-1); % back two to front
            obj.n_left = length(obj.ind_left);
            
            
            obj.q_inf = [10000000; 10000000];
            fields_avoid = cell(veh.sensor.n_front, 1); % Objective avoid field
            for k = 1:veh.sensor.n_front
                %fields_avoid{k} = AvoidObstacleConst(x_vec, y_vec, obj.q_inf, obj.g_max, obj.S);
                fields_avoid{k} = AvoidObstacle(x_vec, y_vec, obj.q_inf, obj.vd);
                fields_avoid{k}.S = obj.S_avoid;
                fields_avoid{k}.R = obj.R_avoid;
            end
            w_avoid = 1;
            weights_avoid = w_avoid*ones(veh.sensor.n_front, 1);
            obj.avoid_field = SummedFields(x_vec, y_vec, fields_avoid, weights_avoid, obj.vd);
            obj.total_field = SummedFields(x_vec, y_vec, {obj.avoid_field, obj.line_vf}, [1;1], obj.vd);
        end
        
        %%%%  Superclass Method Implementation %%%%
        function u = control(obj, t, x, ~)
            % Calculate the line from the sensors
            
            % Calculate the control for the vehicle
            %g_func = @(t_val, x_vec, th)obj.line_vf.getVector(t_val, x_vec, th);
            g_func = @(t_val, x_vec, th)obj.total_field.getVector(t_val, x_vec, th);
            u = obj.vehicle.velocityVectorFieldControl(t, g_func, x);
            
            % find line if needed
            if obj.line_detected && obj.line_lost
                wd = -1; % steer right
                if obj.follow_left
                    wd = -wd; % steer left
                end
                u = obj.vehicle.velocityControl(obj.vd, wd, x);
            end
        end
        
        function processSensors(obj, t, x, agents)
            % Process sensor readings
            processSensors@SingleAgent(obj, t, x, agents);
            q_veh = x(obj.vehicle.q_ind);
            th = x(obj.vehicle.th_ind);
            
            % Load sensor readings into the vector field functions
            % Get obstacle avoidance readings into the vector fields
            if ~isempty(obj.vehicle.xo_latest) && ~isempty(obj.vehicle.yo_latest)
                
                % Extract all points
                q_all = zeros(2, obj.n_sensors);
                ind_inf = zeros(1, obj.n_sensors);
                n_inf = 0;
                for k = 1:obj.n_sensors
                    q = [obj.vehicle.xo_latest(k); obj.vehicle.yo_latest(k)];
                    if isinf(sum(q))
                        q = obj.q_inf;
                        
                        n_inf = n_inf + 1;
                        ind_inf(n_inf) = k;
                    end
                    q_all(:,k) = q;
                end
                ind_inf = ind_inf(1:n_inf);
                
                % Set avoid vector fields
                for k = 1:obj.vehicle.sensor.n_front
                    ind = obj.vehicle.sensor.ind_front(k);
                    obj.avoid_field.fields{k}.x_o = q_all(:,ind);
                end
                
                % Find left wall
                if ~obj.wall_left.initialized
                    % Get front left obstacles
                    q_left = [];
                    n_left_i = 0;
                    for k = 1:obj.vehicle.sensor.n_front_left
                        ind = obj.vehicle.sensor.ind_front_left(k);
                        if isempty(find(ind_inf == ind))
                            n_left_i = n_left_i + 1;
                            q_left(:, n_left_i) = q_all(:, ind);
                        end
                    end
                    
                    % Initialize the left side
                    obj.wall_left.initializePointOnWall(q_left, q_veh, th);
                end
                
                [q_left_wall, ind_left] = obj.wall_left.findContinguousWall(q_all, q_veh);
                %[q_left_wall, ind_left] = obj.wall_left.findContinguousWall(q_all(:, obj.vehicle.sensor.ind_left), q_veh);
                obj.h_left = obj.plotWall(q_left_wall, obj.h_left, 'b');                
                
%                 % Find right wall
%                 if ~obj.wall_right.initialized
%                     % Get front left obstacles
%                     q_right = [];
%                     n_right_i = 0;
%                     for k = 1:obj.vehicle.sensor.n_front_right
%                         ind = obj.vehicle.sensor.ind_front_right(k);
%                         if isempty(find(ind_inf == ind))
%                             n_right_i = n_right_i + 1;
%                             q_right(:, n_right_i) = q_all(:, ind);
%                         end
%                     end
%                     
%                     % Initialize the left side
%                     obj.wall_right.initializePointOnWall(q_right, q_veh, th);
%                 end
%                 [q_right_wall, ind_right] = obj.wall_right.findContinguousWall(q_all, q_veh);
%                 obj.h_right = obj.plotWall(q_right_wall, obj.h_right, 'g');

                  if obj.follow_left
                      q_wall = q_left_wall;
                  else
                      error('Not yet implemented');
                  end
                
                
%                 % Left wall following
%                 q_wall = [];
%                 if obj.follow_left
%                     %for k = 1:obj.vehicle.sensor.n_front_left
%                     count = 0;
%                     for k = 1:obj.n_left
%                         %ind = obj.vehicle.sensor.ind_front_left(k); % Index for the sensor
%                         ind = obj.vehicle.sensor.ind_left(obj.ind_left(k));
%                         q = [obj.vehicle.xo_latest(ind); obj.vehicle.yo_latest(ind)]; % position of sensor reading
%                         count = count + 1; % increment the sensor count
%                         if isinf(sum(q))
%                             if count > 5
%                                 break;
%                             else
%                                 continue;
%                             end
%                         end
%                         
%                         if count > 5 && obj.line_detected
%                             % Create vector from line to sensor point
%                             qs = q - obj.q0;
%                             
%                             % Check for threshold violation
%                             if qs'*obj.v_orth < -obj.gap_thresh
%                                 break;
%                             end                            
%                         end                        
%                         q_wall = [q_wall q];
%                     end
%                 
%                 % Right wall following
%                 else                
%                     for k = 1:obj.vehicle.sensor.n_front_right
%                         ind = obj.vehicle.sensor.ind_front_right(k); % Index for the sensor
%                         q = [obj.vehicle.xo_latest(ind); obj.vehicle.yo_latest(ind)]; % position of sensor reading
%                         if isinf(sum(q))
%                             continue;
%                         end
%                         q_wall = [q_wall q];
%                     end
%                 end
                
                % Calculate the line to follow
                if size(q_wall, 2) > 1
                    % Calculate the line
%                     [obj.q0, obj.vl] = leastSquaresLine(q_wall);  
                    
%                     % Adjust v to point in same direction as vehicle
%                     th = x(obj.vehicle.th_ind); % Orientation angle
%                     orien_vec = [cos(th) sin(th)]; % Orientation vector
%                     if orien_vec*obj.vl < 0 % The line is pointing in the wrong direction
%                         obj.vl = -obj.vl;
%                     end
                    
                    % Calculate line using first and last point
                    obj.q0 = q_wall(:, 1);
                    obj.vl = q_wall(:, end) - obj.q0;
                    obj.vl = obj.vl ./norm(obj.vl);
                    
                    % Adjust v to point in same direction as vehicle
                    th = x(obj.vehicle.th_ind); % Orientation angle
                    orien_vec = [cos(th) sin(th)]; % Orientation vector
                    if orien_vec*obj.vl < 0 % The line is pointing in the wrong direction
                        obj.vl = -obj.vl;
                        obj.q0 = q_wall(:, end);
                    end
                    
                    % Calculate offset orthogonal vector. If follow left
                    % then v needs to be rotated by pi/2 if follow right
                    % then -pi/2
                    R = [0 -1; 1 0]; % right rotation
                    if obj.follow_left 
                        R = -R; % left rotation
                    end
                    obj.v_orth = R*obj.vl;
                    
                    % Calculate a point on the line to be followed
                    ql = obj.q0 + obj.v_orth*obj.offset;
                    
                    % Set the line parameters
                    obj.line_vf.setLineParameters(ql, atan2(obj.vl(2), obj.vl(1)));
                    obj.line_detected = true;
                    obj.line_lost = false;
                    
                    if isempty(obj.h_line)
                        % Plot the line
                        q1 = obj.q0 + obj.vl*5;
                        obj.h_line = plot([obj.q0(1) q1(1)], [obj.q0(2) q1(2)], 'k', 'linewidth', 2);
                        
                        % Plot the data points
%                         obj.h_sensors = plot(q_wall(1,:), q_wall(2,:), 'ko', 'linewidth', 2);
                    else
                        % Plot the line
                        obj.vehicle.plotVehicle();
                        q1 = obj.q0 + obj.vl*5;
                        set(obj.h_line, 'xdata', [obj.q0(1) q1(1)], 'ydata', [obj.q0(2) q1(2)]);
                        
                        % Plot the data points
%                         set(obj.h_sensors, 'xdata', q_wall(1,:), 'ydata', q_wall(2,:));
                    end
                    
                else
                    if obj.line_detected
                        obj.line_lost = true;
                    end
                    warning('Insufficient sensor readings for line detection');
                end
            else
                warning('No sensor readings yet received');
            end
        end
        
        
    end
    
    methods
        function h_plot = plotWall(obj, q_wall, h_plot, color)
            xdata = q_wall(1, :);
            ydata = q_wall(2, :);
            if isempty(h_plot)
                h_plot = plot(xdata, ydata, 'o', 'color', color, 'linewidth', 3);                
            else
                set(h_plot, 'xdata', xdata, 'ydata', ydata);
            end
        end
    end
end

