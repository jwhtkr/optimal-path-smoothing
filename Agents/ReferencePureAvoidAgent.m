classdef ReferencePureAvoidAgent < SingleAgent
    %ReferencePureAvoidAgent Creates an agent that will avoid obstacles
    %using a repulsive force while trying to follow a reference 
    %trajectory
    
    properties(SetAccess = protected)
        % State plotting variable
        h_des_point = []; % Handle to the desired point
        
        % Vector field used to detect obstacle avoidance
        avoid_field % Uses avoid on sensors     
        field_go2goal % Used to influence vehicle towards the goal location
        q_inf = [10000000; 10000000]; % Used to represent the position of a sensor that is not valid
        
        % Line side identifications
        wall_left % Instance of FollowWallBehavior, used for creating vector to be followed on the left side
        wall_right % Instance of FollowWallBehavior, used for creating vector to be followed on the right side
        kv = 10 % Gain on position error for velocity term
                
        % Trajectory variables
        traj_act % The actual trajectory to be followed
        traj_eps % The trajectory followed by the epsilon point
        
        % Current state of the control (slide or follow)
        state = 0; % Default to tracking 
        
        % Store the closest point
        d_close_min = inf; % Distance to the closest point
        q_closest = [inf;inf] % Closest point
        
        %Temporary plotting variables
        h_orbit = []
        h_g2g = []
        h_obs = []
        h_ut = []
    end
    
    properties (Constant)
        % State possibilities
        state_track = 0; % Track the desired trajectory
        state_avoid = 1; % avoid the obstacles
        
        % Obstacle variables
        vd = 1; % Desired velocity
        dist_to_wall = 0.5; % Offset from wall to follow
        dist_cont = 0.3; % Distance considered continuous for wall
        S_avoid = 0.7; % Sphere of influence of avoid
        R_avoid = 0.3; % Radius of full avoid
        switch_threshold = cos(85*pi/180);
        
        % Sensing variables
        n_sensors = 30; % Number of sensor lines
        max_sensor_range = 4; % Maximum range of the sensor
    end
    
    
    methods
        function obj = ReferencePureAvoidAgent(veh, world, traj_act, traj_eps)
            % initialize the scenario
            obj = obj@SingleAgent(veh, world);
            obj.vehicle.sensor.initializeSensor(obj.n_sensors, obj.max_sensor_range);
            
            % Initialize trajectory to be followed
            obj.traj_act = traj_act;
            obj.traj_eps = traj_eps;
            
            % Plotting variables
            x_vec = -1:1:20;
            y_vec = -6:1:10;
            
            % Weights
            w_avoid = 1;
            weights_avoid = w_avoid*ones(veh.sensor.n_front, 1);
            
            % Create a weighted field for the vector field to follow
            fields_avoid = cell(veh.sensor.n_front, 1); % Objective avoid field
            
            
            % Initialize avoid fields
            for k = 1:veh.sensor.n_front
                fields_avoid{k} = AvoidObstacle(x_vec, y_vec, obj.q_inf, obj.vd);
                fields_avoid{k}.S = obj.S_avoid;
                fields_avoid{k}.R = obj.R_avoid;
            end
            
            % Create a combined vector field
            obj.avoid_field = SummedFields(x_vec, y_vec, fields_avoid, weights_avoid, obj.vd);
            
            % Initialize the go to goal behavior
            obj.field_go2goal = GoToGoalField(x_vec, y_vec, obj.q_inf, obj.vd);            
            
            % Initialize the follow wall behaviors
            obj.wall_left = FollowWallBehavior(true, obj.vd, obj.dist_cont, ...
               veh.sensor.ind_front, veh.sensor.ind_left, obj.dist_to_wall, 'b');
            
            obj.wall_right = FollowWallBehavior(false, obj.vd, obj.dist_cont, ...
                  veh.sensor.ind_front, veh.sensor.ind_right, obj.dist_to_wall, 'g');
            
        end
        
        %%%%  Superclass Method Implementation %%%%
        function u = control(obj, t, x, ~)
            % Get the desired values
            [qd, qd_dot, ~] = obj.traj_act.reference_traj(t);
            obj.field_go2goal.x_g = qd;
            
            % Get the avoidance vector (purely for visualization purposes)
            q = x(1:2);
            th = x(obj.vehicle.kinematics.th_ind);
            u_o = obj.calculateRepellantClosestVector(q);
            
            % Calculate the control based upon the state of the system
            if obj.state == obj.state_track
                % Calculate the desired movement of the epsilon point
                [qd_eps, qd_dot_eps, qd_ddot_eps] = obj.traj_eps.reference_traj(t);                
                u = obj.vehicle.pathControl(t, qd_eps, qd_dot_eps, qd_ddot_eps, x);                
                obj.plotVectors(q, [0;0], [0;0], u_o);
                
            elseif obj.state == obj.state_avoid
                % Calculate the vector to follow
                g_func = @(t_val, x_vec, th) obj.avoid_field.getVector(t_val, x_vec, th) + ...
                    obj.field_go2goal.getVector(t_val, x_vec, th); 
                u = obj.vehicle.velocityVectorFieldControl(t, g_func, x);
            else
                error('Invalid state');
            end
            
        end
        
        function processSensors(obj, t, x, agents)
            % Process sensor readings
            processSensors@SingleAgent(obj, t, x, agents);
            
            % Calculate the distance to the closest point seen
            q_veh = x(1:2);
            obj.d_close_min = norm(obj.q_closest - q_veh);
            
            % Load sensor readings into the vector field functions
            % Get obstacle avoidance readings into the vector fields
            if ~isempty(obj.vehicle.xo_latest) && ~isempty(obj.vehicle.yo_latest)
                % Extract all points and keep track of closest point seen
                q_all = zeros(2, obj.n_sensors);
                ind_inf = zeros(1, obj.n_sensors);
                n_inf = 0;
                for k = 1:obj.n_sensors
                    % Extract and store sensor reading
                    q = [obj.vehicle.xo_latest(k); obj.vehicle.yo_latest(k)];
                    if isinf(sum(q))
                        q = obj.q_inf;
                        
                        n_inf = n_inf + 1;
                        ind_inf(n_inf) = k;
                    end
                    q_all(:,k) = q;
                    
                    % Compare to the closest point seen
                    d = norm(q - q_veh);
                    if d < obj.d_close_min
                        obj.d_close_min = d;
                        obj.q_closest = q;
                    end
                end
                ind_inf = ind_inf(1:n_inf);
                
                % Update the finite state machine
                [qd_eps, ~, ~] = obj.traj_eps.reference_traj(t);
                u_t = qd_eps - q_veh;
                obj.updateFSMState(u_t, q_veh);
                
                % Set avoid vector fields
                for k = 1:obj.vehicle.sensor.n_front
                    ind = obj.vehicle.sensor.ind_front(k);                
                    obj.avoid_field.fields{k}.x_o = q_all(:,ind);
                end
                
                % Update the wall following vector fields
                q_veh = x(obj.vehicle.q_ind);
                th = x(obj.vehicle.th_ind);
                
            else
                warning('No sensor readings yet received');
            end
        end
    end
    
    %%% Control functions
    methods (Access = protected)
        function updateFSMState(obj, u_t, q)
        %updateFSMState updates the FSM State
        %   state_track will transition to state_slide if the minimum
        %   obstacle distance threshold is broken
        %
        %   state_slide will transition to state_track under three
        %   conditions:
        %       1. Obstacle avoidance vector field is zero
        %       2. The formulation of the vector of the orthogonal is
        %          not a convex combination of u_o and u_t
        %       3. the Fillipov condition is met 
        %
        
            %obj.plotUt(u_t);
            
            % Store the state at the beginning of the function
            state_start = obj.state; % Used to check if condition changed
            
            % Calculate the obstacle avoidance information and normalize
            % the vectors
            [~, d_o, ~] = obj.calculateRepellantClosestVector(q);            
                            
            % Current state is tracking
            if obj.state == obj.state_track
                % Determine whether to switch to wall following
                if (d_o <= obj.dist_to_wall) 
                    obj.state = obj.state_avoid;
                end
            elseif obj.state == obj.state_avoid 
                if d_o > obj.dist_to_wall
                    obj.state = obj.state_track;
                end
            else
                error('Invalid state');
            end

            % Output message if condition changes
            if state_start ~= obj.state
                if state_start == obj.state_avoid
                    % Output the switch
                    output = 'Track to Avoid';                    
                else
                    output = 'Avoid to track';
                end
                disp(output);
            end
        end
        
        function [u_o, d_min, q_min] = calculateRepellantClosestVector(obj, q)
            %calculateRepellantClosestVector calculates the repelling
            %vector from the closest obstacle given the position q
            %
            % Inputs:
            %   q: 2x1 position point
            %
            % Outputs:
            %   d_min: distance between q and q_min
            %   u_o: vector pointing from q to q_min
            
            % Calculate the min distance to an obstacle 
            d_min = norm(q - obj.q_closest);
            q_min = obj.q_closest;
            
            % Get the repelling vector
            if isinf(d_min)
                u_o = [0;0];
            else
                u_o = q - q_min;
            end
        end
    end
    
    %%% Temporary plotting methods methods
    methods (Access = public)
        function plotVectors(obj, q, g_orb, g_g2g, g_obs)
            % Adjust the obstacle vector to be a unit vector
            g_obs = g_obs ./ norm(g_obs);
            
            % Get the data for each vector
            x_orb = [q(1) q(1) + g_orb(1)]; % Orbit vector
            y_orb = [q(2) q(2) + g_orb(2)];
            x_g2g = [q(1) q(1) + g_g2g(1)]; % g2g vector
            y_g2g = [q(2) q(2) + g_g2g(2)];
            x_obs = [q(1) q(1) + g_obs(1)]; % obs vector
            y_obs = [q(2) q(2) + g_obs(2)];
            
            % Plot the vectors
            if isempty(obj.h_orbit)
                obj.h_orbit = plot(x_orb, y_orb, 'b', 'linewidth', 2); hold on;
                obj.h_g2g = plot(x_g2g, y_g2g, 'k', 'linewidth', 2);
                obj.h_obs = plot(x_obs, y_obs, 'r', 'linewidth', 2);
            else
                set(obj.h_orbit, 'xdata', x_orb, 'ydata', y_orb);
                set(obj.h_g2g, 'xdata', x_g2g, 'ydata', y_g2g);
                set(obj.h_obs, 'xdata', x_obs, 'ydata', y_obs);
            end
            
        end
        
        function plotUt(obj, ut)
            if isempty(obj.h_ut)
                ax = gca;
                
                figure;
                subplot(2,1,1);
                obj.h_ut(1) = plot(1, ut(1));
                
                subplot(2,1,2);
                obj.h_ut(2) = plot(1, ut(2));
                
                axes(ax);
            else
                xdata = get(obj.h_ut(1), 'xdata');
                xdata = [xdata xdata(end)+1];
                
                y_ut1 = [get(obj.h_ut(1), 'ydata') ut(1)];
                y_ut2 = [get(obj.h_ut(2), 'ydata') ut(2)];
                
                set(obj.h_ut(1), 'xdata', xdata, 'ydata', y_ut1);
                set(obj.h_ut(2), 'xdata', xdata, 'ydata', y_ut2);
                
            end
        end
    end
    
end