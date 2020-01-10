classdef ReferenceAvoidScenario < Scenario
    %ReferenceTrackingUnicycleScenario This scenario has a robot
    %follow a sinusoidal path input
    
    properties
        % Sinusoid properties
        a = 2 % Amplitude
        f = 1 % Frequency (in radians)
        
        % State plotting variable
        h_des_point = []; % Handle to the desired point
        
        % Vector field for obstacle avoidance
        orbit_field % Uses orbit on sensors
        avoid_field % Uses avoid on sensors
        q_inf
        n_sensors; % Stores the number of sensors
        
        % Current state of the control (slide or follow)
        state = 1; % Default to tracking        
    end
    
    properties (Constant)
        % State possibilities
        state_track = 1;
        state_slide = 0;
        
        % Obstacle variables
        g_max = 1; % Maximum magnitude of the vector to be followed
        S = 4; % Sphere of influence
        R = 2; % Radius of orbit        
    end
    
    
    methods
        function obj = ReferenceAvoidScenario(veh, world)
            % initialize the scenario
            obj = obj@Scenario(veh, world, true); 
            
            % Plotting variables
            x_vec = -1:1:20;
            y_vec = -6:1:10;
            
            % Obstacle avoidance variables - orbit
            k_conv = 0.5; % Convergence gain
            
            % Weights
            w_orbit = 1;
            w_avoid = 1;
            weights_orbit = w_orbit*ones(veh.sensor.n_lines, 1);
            weights_avoid = w_avoid*ones(veh.sensor.n_lines, 1);
            
            % Create a weighted field for the vector field to follow
            fields_orbit = cell(veh.sensor.n_lines, 1); % object orbit field
            fields_avoid = cell(veh.sensor.n_lines, 1); % Objective avoid field
            q_inf = [10000000; 10000000];
            for k = 1:veh.sensor.n_lines
                fields_orbit{k} = OrbitAvoidField(x_vec, y_vec, q_inf, obj.R, obj.g_max, k_conv, obj.S);
                fields_avoid{k} = AvoidObstacleConst(x_vec, y_vec, q_inf, obj.g_max, obj.S);                
            end
            
            % Create a combined vector field
            obj.orbit_field = SummedFields(x_vec, y_vec, fields_orbit, weights_orbit, obj.g_max);
            obj.avoid_field = SummedFields(x_vec, y_vec, fields_avoid, weights_avoid, obj.g_max);
            
            % Initialize sensors
            obj.n_sensors = veh.sensor.n_lines;
            obj.q_inf = q_inf;
        end
    
        %%%%  Abstract Method Implementation %%%%
        function u = control(obj, t, x)
            % Get obstacle avoidance readings into the vector fields
            if ~isempty(obj.vehicle.xo_latest) && ~isempty(obj.vehicle.yo_latest)
                for k = 1:obj.n_sensors
                    q = [obj.vehicle.xo_latest(k); obj.vehicle.yo_latest(k)];
                    if isinf(sum(q))
                        q = obj.q_inf;
                    end
                    obj.orbit_field.fields{k}.x_o = q;
                    obj.avoid_field.fields{k}.x_o = q;
                end
            else
                warning('No sensor readings yet received');
            end
            
            % Get the desired values
            [qd, qd_dot, qd_ddot] = obj.SineReference(t);
            
            % Get the avoidance vector
            q_eps = obj.vehicle.calculateEpsilonPoint(t, x);
            th = x(obj.vehicle.kinematics.th_ind);
            u_o = obj.avoid_field.getVector(t, q_eps, th);
            
            % Calculate the epsilon point control for trajectory tracking
            u_t = obj.vehicle.epsilonPathControl(t, qd, qd_dot, qd_ddot, x);
            
            % Determine the FSM state and desired direction
               %%%TODO: May want to change qd_ddot back to u_t in updating
               %%%state
            obj.updateFSMState(u_o, qd_ddot, q_eps);
            u_d = obj.calculateDesiredAcceleration(t, q_eps, u_t, th);
            
            % Threshold u_d so that it is not larger than u_t
            if norm(u_d) > norm(u_t)
                u_d = (norm(u_t)/norm(u_d)) .* u_d;
            end
            
            % Calculate the vehicle control
            u = obj.vehicle.epsilonToVehicleControl(t, x, u_d);
        end
        
        function [u_s, a] = calculateSlidingModeControl(obj, u_o, u_t)
           %calculateSlidingModeControl returns a vector orthogonal to u_o
           %(obstacle avoidance vector) which is a convex combination of
           %u_o and u_t (the trajectory vector)
           
           % Calculate the convex combination coefficient
           a = -(u_o'*u_t) / (u_o'*u_o - u_o'*u_t); 
           
           % Create the combination
           u_s = a*u_o + (1-a)*u_t;
        end
        
        function updateFSMState(obj, u_o, u_t, q)
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
        
            % If statement for each possible state
            if obj.state == obj.state_track
                % Calculate the min distance to an obstacle using the epsilon
                % point
                d_min = inf;
                for k = 1:obj.n_sensors
                    d = norm(obj.avoid_field.fields{k}.x_o - q);
                    if d < d_min
                        d_min = d;
                    end
                end
                
                % If the min distance is less than transition distance,
                % then transition
                if d_min <= obj.R
                    obj.state = obj.state_slide;
                end                
            elseif obj.state == obj.state_slide
                % State will transition back to state_track under three
                % conditions
                %   1. Obstacle avoidance vector field is zero
                %   2. The formulation of the vector of the orthogonal is
                %      not a convex combination of u_o and u_t
                %   3. the Fillipov condition is met
                
                % 1. Obstacle avoidance vector field is zero
                if norm(u_o) == 0
                    obj.state = obj.state_track;
                    return;
                end
                
                % Calculate the sliding mode control
                [u_h, alpha] = obj.calculateSlidingModeControl(u_o, u_t);
                
                % 2. The formulation of the vector of the orthogonal is
                %    not a convex combination of u_o and u_t
                if alpha < 0 || alpha > 1
                    obj.state = obj.state_track;
                    return;
                end
                
                % Calculate parameters needed to evaluate Fillipov
                % condition
                n = u_o; % Normal vector to 
                n_dot_uo = n'*u_o; % product for decision
                n_dot_ut = n'*u_t;
                
                % 3. the Fillipov condition is met
                if n_dot_uo*n_dot_ut > 0
                    obj.state = obj.state_track;
                    return;
                end                
            else
                error('Invalid state');
            end
        end
        
        function u_d = calculateDesiredAcceleration(obj, t, q, u_t, th)
            % Desired force vector
            %   The desired force vector is state dependent:
            %       state_track: u_d = u_t
            %       state_slide: u_d is the sliding mode direction. Note
            %       that we do not use the actual sliding mode direction as
            %       it has not restoring force. Instead, we use a behavior
            %       which will move us into the sliding mode direction
            
            if obj.state == obj.state_track
                u_d = u_t;
            elseif obj.state == obj.state_slide
                u_d = obj.orbit_field.getVector(t, q, th);                
            else
                error('Invalid state');
            end
        end
        
        %%%% Plotting methods - Add reference %%%%
        function plotState(obj, t)
            plotState@Scenario(obj,t);
            
            if isempty(obj.h_des_point)
                warning('Attempting to plot state without initializing plots');
                return;
            end
            
            % Plot the desired position
            qd = obj.SineReference(t);
            set(obj.h_des_point, 'xdata', qd(1), 'ydata', qd(2));
        end
        
        function initializeStatePlot(obj)
            initializeStatePlot@Scenario(obj);
            
            % Get and plot the desired trajectory
            qd_traj = obj.SineReference([obj.t0:obj.dt:obj.tf]);
            plot(qd_traj(1,:), qd_traj(2,:), 'r'); hold on;
            
            % Plot the desired trajectory point
            obj.h_des_point = plot(qd_traj(1,1), qd_traj(2,1), 'ro', 'linewidth', 2);
        end
        
        function plotResults(obj)
            % Create the figure and get additional data
            figure;
            qd_traj = obj.SineReference(obj.tmat);
            
            % Plot x1 and desired
            subplot(4,1,1);
            plot(obj.tmat, qd_traj(1,:), 'r', 'linewidth', 2); hold on;
            plot(obj.tmat, obj.xmat(1,:), 'b', 'linewidth', 2);
            ylabel('x_1(t)');
            legend('Desired', 'Actual');
            
            % Plot x2 and desired
            subplot(4,1,2);
            plot(obj.tmat, qd_traj(2,:), 'r', 'linewidth', 2); hold on;
            plot(obj.tmat, obj.xmat(2,:), 'b', 'linewidth', 2);
            ylabel('x_2(t)');

            % Calculate the error and control vs time
            err = zeros(1,length(obj.tmat));
            ctrl = zeros(2,length(obj.tmat));
            for k = 1:length(obj.tmat)
                err(k) = norm(qd_traj(1:2,k) - obj.xmat(1:2,k));
                ctrl(:,k) = obj.control(obj.tmat(k), obj.xmat(:,k));
            end
            
            % Plot the error vs time
            subplot(4,1,3);
            plot(obj.tmat, err, 'r', 'linewidth', 3);
            ylabel('Error');
            
            % Plot the control vs time
            subplot(4,1,4);
            plot(obj.tmat, ctrl(1,:), 'g', 'linewidth', 3); hold on;
            plot(obj.tmat, ctrl(2,:), 'b', 'linewidth', 3); 
            ylabel('Inputs');
            legend('u_v', 'u_\omega');            
            xlabel('Time (s)');
        end
        
        
        %%%% Reference trajectory methods %%%%
        function [qd, qd_dot, qd_ddot] = SineReference(obj, t)
            % Get length of the time vector
            n = length(t);
            t = reshape(t, 1, n);

            % Get the reference
            qd = [t; obj.a.*sin(obj.f.*t)];                     % Desired position
            qd_dot = [ones(1,n); obj.a*obj.f*cos(obj.f.*t)];        % Desired velocity vector
            qd_ddot = [zeros(1,n); -obj.a.*obj.f^2.*sin(obj.f.*t)]; % Desired acceleration vector
        end
    end
end

