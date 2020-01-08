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
        vector_field
        avoid_indices
        q_inf
        n_sensors; % Stores the number of sensors
    end
    
    methods
        function obj = ReferenceAvoidScenario(veh, world)
            % initialize the scenario
            obj = obj@Scenario(veh, world, true); 
            
            % Plotting variables
            x_vec = -1:1:20;
            y_vec = -6:1:10;
            
            % Vehicle variables
            v_max = 3;
            
            % Obstacle avoidance variables - orbit
            S = 3; % Sphere of influence
            R = 2; % Radius of orbit
            k_conv = 0.5; % Convergence gain
            
            % Obstacle avoidance variables - barrier
            S_b = 2.0; % Sphere of influence of barrier
            R_b = 0.5; % Radius of full influence
            
            % Weights
            w_avoid = 0;
            w_barrier = 5;
            weights = zeros(veh.sensor.n_lines*2, 1);
            weights(1:veh.sensor.n_lines) = w_avoid;
            weights(1+veh.sensor.n_lines:end) = w_barrier;
            
            % Create a weighted field for the vector field to follow
            fields = cell(veh.sensor.n_lines*2, 1); % object avoidance
            avoid_indices = 1:veh.sensor.n_lines;
            q_inf = [10000000; 10000000];
            for k = avoid_indices
                fields{k} = OrbitAvoidField(x_vec, y_vec, q_inf, R, v_max, k_conv, S);  
                fields{k+veh.sensor.n_lines} = AvoidObstacle(x_vec, y_vec, q_inf, v_max);
                fields{k+veh.sensor.n_lines}.S = S_b;
                fields{k+veh.sensor.n_lines}.R = R_b;
            end
            
            % Create a combined vector field
            obj.vector_field = SummedFields(x_vec, y_vec, fields, weights, v_max);
            
            % Initialize sensors
            obj.n_sensors = veh.sensor.n_lines;
            obj.avoid_indices = avoid_indices;
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
                    obj.vector_field.fields{obj.avoid_indices(k)}.x_o = q;
                    obj.vector_field.fields{obj.avoid_indices(k)+obj.n_sensors}.x_o = q;
                end
            else
                warning('No sensor readings yet received');
            end
            
            % Get the desired values
            [qd, qd_dot, qd_ddot] = obj.SineReference(t);
            
            % Get the vector
            g = @(t_val, x_vec, th)obj.vector_field.getVector(t_val, x_vec, th);
            
            % Calculate the control
            u = obj.vehicle.pathVectorControl(t, qd, qd_dot, qd_ddot, g, x);
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

