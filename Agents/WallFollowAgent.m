classdef WallFollowAgent < SingleAgent
    %WallFollowAgent Creates an agent that will follow a wall
    
    properties
        % Vector field for line following
        follow_left % true => follow line on left of vehicle, false => follow line on right of vehicle
        
        % Sensor processing variables
        q_inf = [10000000; 10000000]; % Insert for a sensor that did not detect
        
        % Line side identifications
        wall % Instance of FollowWallBehavior, used for creating vector to be followed
    end
    
    properties (Constant)
        % Line variables
        vd = 1; % Desired velocity
        dist_to_wall = 0.5; % Offset from wall to follow
        dist_cont = 0.3; % Distance considered continuous for wall
                
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
            
            % Initialize the follow wall behavior
            if follow_left
                obj.wall = FollowWallBehavior(true, obj.vd, obj.dist_cont, ...
                    veh.sensor.ind_front, veh.sensor.ind_left, obj.dist_to_wall, 'b');
            else
                obj.wall = FollowWallBehavior(false, obj.vd, obj.dist_cont, ...
                    veh.sensor.ind_front, veh.sensor.ind_right, obj.dist_to_wall, 'g');
            end
        end
        
        %%%%  Superclass Method Implementation %%%%
        function u = control(obj, t, x, ~)
            % Calculate the line from the sensors
            
            % Calculate the control for the vehicle
            g_func = @(t_val, x_vec, th)obj.wall.getVector(t_val, x_vec, th);
            u = obj.vehicle.velocityVectorFieldControl(t, g_func, x);
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
                
                % Update the behavior vector fields
                obj.wall.updateVectorFields(q_all, ind_inf, q_veh, th);                
            else
                warning('No sensor readings yet received');
            end
        end
    end
end

