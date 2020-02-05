classdef WallFollowAgent < SingleAgent
    %WallFollowAgent Creates an agent that will follow a wall
    
    properties
        % Vector field for line following
        line_vf % Used to create a line
        follow_left % true => follow line on left of vehicle, false => follow line on right of vehicle
        
        avoid_field % Uses avoid on sensors
        total_field % Summation of line_vf and avoid_vf
        
        n_sensors; % Stores the number of sensors
        
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
    end
    
    properties (Constant)
        % Line variables
        vd = 1; % Desired velocity
        slope = 1; % Slope of sigmoid to the line
        offset = 1; % Offset from wall to follow
        gap_thresh = 20; % Threshold the distance to gap
        
        
        S_avoid = 1; % Sphere of influence of avoid
        R_avoid = 0.25; % Radius of full avoid
    end
    
    
    methods
        function obj = WallFollowAgent(veh, world, follow_left)
            % initialize the scenario
            obj = obj@SingleAgent(veh, world); 
            obj.follow_left = follow_left;
            
            % Plotting variables
            x_vec = -1:1:20;
            y_vec = -6:1:10;
            
            % Initialize vector field
            obj.line_vf = LineVectorField(x_vec, y_vec, [0;0], 0, obj.slope, obj.vd);
            
            % Initialize sensors
            obj.n_sensors = veh.sensor.n_lines;
            
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
            
            % Load sensor readings into the vector field functions
            % Get obstacle avoidance readings into the vector fields
            if ~isempty(obj.vehicle.xo_latest) && ~isempty(obj.vehicle.yo_latest)
                
                % Set avoid vector fields
                for k = 1:obj.vehicle.sensor.n_front
                    ind = obj.vehicle.sensor.ind_front(k);                
                    q = [obj.vehicle.xo_latest(ind); obj.vehicle.yo_latest(ind)];
                    if isinf(sum(q))
                        q = obj.q_inf;
                    end
                    obj.avoid_field.fields{k}.x_o = q;
                end
                
                % Left wall following
                q_wall = [];
                if obj.follow_left
                    %for k = 1:obj.vehicle.sensor.n_front_left
                    count = 0;
                    for k = 1:obj.n_left
                        %ind = obj.vehicle.sensor.ind_front_left(k); % Index for the sensor
                        ind = obj.vehicle.sensor.ind_left(obj.ind_left(k));
                        q = [obj.vehicle.xo_latest(ind); obj.vehicle.yo_latest(ind)]; % position of sensor reading
                        count = count + 1; % increment the sensor count
                        if isinf(sum(q))
                            if count > 5
                                break;
                            else
                                continue;
                            end
                        end
                        
                        if count > 5 && obj.line_detected
                            % Create vector from line to sensor point
                            qs = q - obj.q0;
                            
                            % Check for threshold violation
                            if qs'*obj.v_orth < -obj.gap_thresh
                                break;
                            end                            
                        end                        
                        q_wall = [q_wall q];
                    end
                
                % Right wall following
                else                
                    for k = 1:obj.vehicle.sensor.n_front_right
                        ind = obj.vehicle.sensor.ind_front_right(k); % Index for the sensor
                        q = [obj.vehicle.xo_latest(ind); obj.vehicle.yo_latest(ind)]; % position of sensor reading
                        if isinf(sum(q))
                            continue;
                        end
                        q_wall = [q_wall q];
                    end
                end
                
                % Calculate the line to follow
                if size(q_wall, 2) > 1
                    % Calculate the line
                    [obj.q0, obj.vl] = leastSquaresLine(q_wall);                    
                    
                    % Adjust v to point in same direction as vehicle
                    th = x(obj.vehicle.th_ind); % Orientation angle
                    orien_vec = [cos(th) sin(th)]; % Orientation vector
                    if orien_vec*obj.vl < 0 % The line is pointing in the wrong direction
                        obj.vl = -obj.vl;
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
                        obj.h_sensors = plot(q_wall(1,:), q_wall(2,:), 'ko', 'linewidth', 2);
                    else
                        % Plot the line
                        obj.vehicle.plotVehicle();
                        q1 = obj.q0 + obj.vl*5;
                        set(obj.h_line, 'xdata', [obj.q0(1) q1(1)], 'ydata', [obj.q0(2) q1(2)]);
                        
                        % Plot the data points
                        set(obj.h_sensors, 'xdata', q_wall(1,:), 'ydata', q_wall(2,:));
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
end

