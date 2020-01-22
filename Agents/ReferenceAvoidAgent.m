classdef ReferenceAvoidAgent < SingleAgent
    %ReferenceAvoidAgent Creates an agent that will avoid obstacles while
    %trying to follow a reference trajectory
    
    properties
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
        n_sensors; % Stores the number of sensors
        
        % Trajectory variables
        clothoid
        
        % Current state of the control (slide or follow)
        state = 2; % Default to tracking        
    end
    
    properties (Constant)
        % State possibilities
        state_track = 2; % Track the desired trajectory
        state_slide_cw = 1; % orbit the obstacles on the right
        state_slide_ccw = 0; % orbit the obstacles on the left
        
        % Obstacle variables
        g_max = 1; % Maximum magnitude of the vector to be followed
        k_max = 5; % Maximum curvature
        sig_max = 5; % Maximum change in curvature
        S = 4; % Sphere of influence of orbit
        R_orbit = 1; % Radius of orbit
        S_avoid = 4; % Sphere of influence of avoid
        R_avoid = 1; % Radius of full avoid
    end
    
    
    methods
        function obj = ReferenceAvoidAgent(veh, world, waypoints, dt)
            % initialize the scenario
            obj = obj@SingleAgent(veh, world); 
            
            % Initialize trajectory to be followed
            obj.clothoid = CCPathGenerator(waypoints, obj.g_max, dt, obj.k_max, obj.sig_max);
            
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
            obj.n_sensors = veh.sensor.n_lines;
            obj.q_inf = q_inf;
        end
        
        %%%%  Superclass Method Implementation %%%%
        function u = control(obj, t, x, ~)
            % Get the desired values
            [qd, qd_dot, qd_ddot] = obj.ReferenceTraj(t);
            
            % Set the desired point as the goal point
            obj.field_go2goal.x_g = qd;
            
            % Get the avoidance vector
            q_eps = obj.vehicle.calculateEpsilonPoint(t, x);
            th = x(obj.vehicle.kinematics.th_ind);
            %u_o = obj.avoid_field.getVector(t, q_eps, th);
            u_o = obj.avoid_field.getVector(t, x(1:2), th);
            
            % Calculate the epsilon point control for trajectory tracking
            u_t_switch = qd-q_eps;
            if norm(u_t_switch) < 2*obj.vehicle.getEpsilon(t)
                u_t_switch = qd_dot;
            end
            
            % Determine the FSM state and desired direction
               %%%TODO: May want to change qd_ddot back to u_t in updating
               %%%state
            u_h = obj.updateFSMState(u_o, u_t_switch, q_eps);

            if obj.state == obj.state_track
                u = obj.vehicle.pathControl(t, qd, qd_dot, qd_ddot, x);
                
            elseif obj.state == obj.state_slide_ccw
                g_func = @(t_val, x_vec, th)obj.orbit_field_ccw.getVector(t_val, x_vec, th);                
                u = obj.vehicle.velocityVectorFieldControl(t, g_func, x);
            elseif obj.state == obj.state_slide_cw
                g_func = @(t_val, x_vec, th)obj.orbit_field_cw.getVector(t_val, x_vec, th);
                u = obj.vehicle.velocityVectorFieldControl(t, g_func, x);
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
    
    %%% Reference trajectory methods
    methods (Access = public)
        function [qd, qd_dot, qd_ddot] = ReferenceTraj(obj, t)
            [qd, qd_dot, qd_ddot] = obj.WaypointReference(t);
        end
        
    end
    methods (Access = protected)
        function [qd, qd_dot, qd_ddot] = SineReference(obj, t)
            
            if true
                [qd, qd_dot, qd_ddot] = obj.WaypointReference(t);
                return;
            end
            if false
                [qd, qd_dot, qd_ddot] = obj.LineReference(t);
                return;
            end
            
            % Get length of the time vector
            n = length(t);
            t = reshape(t, 1, n);

            % Get the reference
            qd = [t; obj.a.*sin(obj.f.*t)];                     % Desired position
            qd_dot = [ones(1,n); obj.a*obj.f*cos(obj.f.*t)];        % Desired velocity vector
            qd_ddot = [zeros(1,n); -obj.a.*obj.f^2.*sin(obj.f.*t)]; % Desired acceleration vector
        end
        
        function [qd, qd_dot, qd_ddot] = LineReference(obj, t)
            % Get length of the time vector
            n = length(t);
            t = reshape(t, 1, n);
            
            qd = [t; zeros(1,n)];
            qd_dot = [ones(1,n); zeros(1,n)];
            qd_ddot = [zeros(1,n); zeros(1,n)];
            
        end
        
        function [qd, qd_dot, qd_ddot] = WaypointReference(obj, t)
            [qd, qd_dot, qd_ddot] = obj.clothoid.reference_traj(t);
        end
    end
    
end

