classdef ReferenceAvoidAgent < SingleAgent
    %ReferenceAvoidAgent Creates an agent that will avoid obstacles while
    %trying to follow a reference trajectory
    
    properties(SetAccess = protected)
        % Sinusoid properties
        a = 2 % Amplitude
        f = 1 % Frequency (in radians)
        
        % State plotting variable
        h_des_point = []; % Handle to the desired point
        
        % Vector field for obstacle avoidance
        orbit_field_ccw % Uses sensors on the left of vehicle for counter-clockwise orbit
        orbit_field_cw % Uses sensors on the right of the vehicle for clockwise orbit
        avoid_field % Uses avoid on sensors
        field_go2goal % Used to influence vehicle towards the goal location
        w_g2g = 1 % weight on the go to goal field
        
        q_inf
        
        % Trajectory variables
        traj_act % The actual trajectory to be followed
        traj_eps % The trajectory followed by the epsilon point
        
        % Current state of the control (slide or follow)
        state = 2; % Default to tracking 
        
        
        %Temporary plotting variables
        h_orbit = []
        h_g2g = []
        h_obs = []
        h_ut = []
    end
    
    properties (Constant)
        % State possibilities
        state_track = 2; % Track the desired trajectory
        state_slide_cw = 1; % orbit the obstacles on the right
        state_slide_ccw = 0; % orbit the obstacles on the left
        
        % Obstacle variables
        g_max = 1; % Maximum magnitude of the vector to be followed
        S = 4; % Sphere of influence of orbit
        R_orbit = 1; % Radius of orbit
        S_avoid = 4; % Sphere of influence of avoid
        R_avoid = 1; % Radius of full avoid
        
        % Sensing variables
        n_sensors = 30; % Number of sensor lines
        max_sensor_range = 4; % Maximum range of the sensor
    end
    
    
    methods
        function obj = ReferenceAvoidAgent(veh, world, traj_act, traj_eps)
            % initialize the scenario
            obj = obj@SingleAgent(veh, world);
            obj.vehicle.sensor.initializeSensor(obj.n_sensors, obj.max_sensor_range);
            
            % Initialize trajectory to be followed
            obj.traj_act = traj_act;
            obj.traj_eps = traj_eps;
            
            % Plotting variables
            x_vec = -1:1:20;
            y_vec = -6:1:10;
            
            % Obstacle avoidance variables - orbit
            k_conv = 0.5; % Convergence gain
            
            % Weights
            w_orbit = 1;
            w_avoid = 1;
            weights_avoid = w_avoid*ones(veh.sensor.n_front, 1);
            
            % Create a weighted field for the vector field to follow
            fields_ccw = cell(veh.sensor.n_left, 1); % object orbit field for ccw orbit
            fields_cw = cell(veh.sensor.n_right, 1); % object orbit field for cw orbit
            fields_avoid = cell(veh.sensor.n_front, 1); % Objective avoid field
            q_inf = [10000000; 10000000];
            
            % Initialize avoid fields
            for k = 1:veh.sensor.n_front
                %fields_avoid{k} = AvoidObstacleConst(x_vec, y_vec, q_inf, obj.g_max, obj.S);
                fields_avoid{k} = AvoidObstacle(x_vec, y_vec, q_inf, obj.g_max);
                fields_avoid{k}.S = obj.S_avoid;
                fields_avoid{k}.R = obj.R_avoid;
            end
            
            % Initialize ccw fields
            for k = 1:veh.sensor.n_left
                fields_ccw{k} = OrbitAvoidField(x_vec, y_vec, q_inf, obj.R_orbit, obj.g_max, k_conv, obj.S);
            end
            
            % Initailize cw fields
            for k = 1:veh.sensor.n_right
                fields_cw{k} = OrbitAvoidField(x_vec, y_vec, q_inf, obj.R_orbit, obj.g_max, k_conv, obj.S);
            end
            
            % Add in a go to goal field for the orbit avoid
            obj.field_go2goal = GoToGoalField(x_vec, y_vec, q_inf, obj.g_max);            
            
            % Create a combined vector field
            obj.orbit_field_ccw = SummedFields(x_vec, y_vec, fields_ccw, w_orbit.*ones(veh.sensor.n_left,1), obj.g_max);
            obj.orbit_field_cw = SummedFields(x_vec, y_vec, fields_cw, w_orbit.*ones(veh.sensor.n_left,1), obj.g_max);
            obj.avoid_field = SummedFields(x_vec, y_vec, fields_avoid, weights_avoid, obj.g_max);
            
            % Initialize sensors
            obj.q_inf = q_inf;
        end
        
        %%%%  Superclass Method Implementation %%%%
        function u = control(obj, t, x, ~)
            % Get the desired values
            [qd, qd_dot, ~] = obj.traj_act.reference_traj(t);
            [qd_eps, qd_dot_eps, qd_ddot_eps] = obj.traj_eps.reference_traj(t);
            
            % Set the desired point as the goal point
            obj.field_go2goal.x_g = qd;
            
            % Get the avoidance vector
            q_eps = obj.vehicle.calculateEpsilonPoint(t, x);
            th = x(obj.vehicle.kinematics.th_ind);
            %u_o = obj.avoid_field.getVector(t, q_eps, th);
            u_o = obj.avoid_field.getVector(t, x(1:2), th);
            
            % Calculate the epsilon point control for trajectory tracking
            %u_t_switch = qd-q_eps;
            u_t_switch = qd_eps-x(1:2);
            if false% norm(u_t_switch) < 2*obj.vehicle.getEpsilon(t)
                u_t_switch = qd_dot;
            end
            
            % Determine the FSM state and desired direction
               %%%TODO: May want to change qd_ddot back to u_t in updating
               %%%state
            u_h = obj.updateFSMState(u_o, u_t_switch, q_eps);

            if obj.state == obj.state_track
                % Calculate the desired movement of the epsilon point
                [qd_eps, qd_dot_eps, qd_ddot_eps] = obj.traj_eps.reference_traj(t);                
                u = obj.vehicle.pathControl(t, qd_eps, qd_dot_eps, qd_ddot_eps, x);
                
                obj.plotVectors(x(1:2), [0;0], [0;0], u_o);
                
            elseif obj.state == obj.state_slide_ccw
                g_func = @(t_val, x_vec, th)obj.orbit_field_ccw.getVector(t_val, x_vec, th) ; % + ...
                    %obj.field_go2goal.getVector(t_val, x_vec, th);                
                u = obj.vehicle.velocityVectorFieldControl(t, g_func, x);
                
                obj.plotVectors(x(1:2), obj.orbit_field_ccw.getVector(t, x(1:2), th), ...
                    obj.field_go2goal.getVector(t, x(1:2), th), u_o);
            elseif obj.state == obj.state_slide_cw
                g_func = @(t_val, x_vec, th)obj.orbit_field_cw.getVector(t_val, x_vec, th) ; %+ ...
                    %obj.field_go2goal.getVector(t_val, x_vec, th);
                u = obj.vehicle.velocityVectorFieldControl(t, g_func, x);
                
                obj.plotVectors(x(1:2), obj.orbit_field_cw.getVector(t, x(1:2), th), ...
                    obj.field_go2goal.getVector(t, x(1:2), th), u_o);
            else
                error('Invalid state');
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
                
                % Set fields for ccw motion
                for k = 1:obj.vehicle.sensor.n_left
                    ind = obj.vehicle.sensor.ind_left(k); % Index for the sensor
                    q = [obj.vehicle.xo_latest(ind); obj.vehicle.yo_latest(ind)]; % position of sensor reading
                    if isinf(sum(q))
                        q = obj.q_inf;
                    else
                        q = q;
                    end
                    obj.orbit_field_ccw.fields{k}.x_o = q;
                end
                
                % Set fields for cw motion
                for k = 1:obj.vehicle.sensor.n_right
                    ind = obj.vehicle.sensor.ind_right(k); % Index for the sensor
                    q = [obj.vehicle.xo_latest(ind); obj.vehicle.yo_latest(ind)]; % position of sensor reading
                    if isinf(sum(q))
                        q = obj.q_inf;
                    else
                        q = q;
                    end
                    obj.orbit_field_cw.fields{k}.x_o = q;
                end
            else
                warning('No sensor readings yet received');
            end
        end
    end
    
    %%% Control functions
    methods (Access = protected)
        function [u_s, a] = calculateSlidingModeControl(obj, u_o, u_t)
           %calculateSlidingModeControl returns a vector orthogonal to u_o
           %(obstacle avoidance vector) which is a convex combination of
           %u_o and u_t (the trajectory vector)
           
           % Calculate the convex combination coefficient
           a = -(u_o'*u_t) / (u_o'*u_o - u_o'*u_t); 
           
           % Create the combination
           u_s = a*u_o + (1-a)*u_t;
        end
        
        function u_h = updateFSMState(obj, u_o, u_t, q)
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
        
            obj.plotUt(u_t);
            
            % 1. Obstacle avoidance vector field is zero
            state_start = obj.state; % Used to check if condition changed
            if norm(u_o) == 0
                obj.state = obj.state_track;
                u_h = [0;0];
            else
                % Calculate the sliding mode control
                [u_h, alpha] = obj.calculateSlidingModeControl(u_o, u_t);
                alpha_valid = alpha >= 0 && alpha <= 1;

                % Calculate the min distance to an obstacle using the epsilon
                % point
                d_min = inf;
                for k = 1:obj.vehicle.sensor.n_front
                    d = norm(obj.avoid_field.fields{k}.x_o - q);
                    if d < d_min
                        d_min = d;
                    end
                end

                % Calculate parameters needed to evaluate Fillipov
                % condition
                n = u_o; % Normal vector to 
                n_dot_uo = n'*u_o; % product for decision
                n_dot_ut = n'*u_t;

                % Evaluate Fillipov condition
                if n_dot_uo*n_dot_ut > 0
                    track = true;
                else
                    track = false;
                end 

                %%% State machine
                if obj.state == obj.state_track
                    if alpha_valid && d_min <= obj.R_orbit && (~track)

                        % Determine which side to slide
                        th_uh = atan2(u_h(2), u_h(1));
                        th_ut = atan2(u_t(2), u_t(1));
                        th_e = th_uh - th_ut;
                        th_e = atan2(sin(th_e), cos(th_e));

                        if th_e > 0
                            obj.state = obj.state_slide_cw;
                        else
                            obj.state = obj.state_slide_ccw;
                        end
                    end
                elseif obj.state == obj.state_slide_cw || obj.state == obj.state_slide_ccw
                    if ~alpha_valid || track
                        obj.state = obj.state_track;
                    end
                else
                    error('Invalid state');
                end
            end

            % Output message if condition changes
            if state_start ~= obj.state
                if state_start == obj.state_track
                    % Output the switch
                    if obj.state == obj.state_slide_cw
                        output = 'Track to slide - right';
                    else
                        output = 'Track to slide - left';
                    end

                else
                    output = 'Slide to track';
                end
                disp(output);
            end

        end
    end
    
    %%% Temporary plotting methods methods
    methods (Access = public)
        function plotVectors(obj, q, g_orb, g_g2g, g_obs)
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