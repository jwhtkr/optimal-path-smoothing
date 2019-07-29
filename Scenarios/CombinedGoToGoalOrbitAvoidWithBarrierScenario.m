classdef CombinedGoToGoalOrbitAvoidWithBarrierScenario < VectorFieldScenario
    %CombinedGoToGoalOrbitAvoidScenario Combines a simple go to goal with
    %orbit-based obstacle avoidance
    
    properties
        avoid_indices % Stores the indices for the avoidance fields
        q_inf; % Large number to use for placing an obstacle infinitely far away
        n_sensors; % Stores the number of sensors
        x_vec;
        y_vec;
        v_max;
        
        % Go to goal variables
        x_g; % Stores the location goal of the robot
        goals;
        goal_ind = 1;
        goal_switch = 0.3; % Distance away from a goal for switching to occur
        n_goals; % The number of goals
        g2g_field_index; % The index of the go-to-goal within the array of vector fields
        h_goal_plot = [] % Plot the goal location
    end
    
    methods
        function obj = CombinedGoToGoalOrbitAvoidWithBarrierScenario(veh, control_type)
            % Plotting variables
            x_vec = -1:1:16; % size of the vector field arrow graph limits
            y_vec = -6:1:3; % size of the vector field arrow graph limits
            
            % Vehicle variables
            v_max = 0.5;
                        
            % Go to goal variables
            goals = [  3,0,3,   3,  5.75, 6.5, 5.5, 6.5, 8.5, 9.5;
                     1.5,0,1.5,-6.5,  -6,  -1,  -6, -6, 0.5,  -6]; 
%             goals = [  6, 8.5, 9.5;
%                       -6, 0.5,  -6]; 
%             broke through wall, was from straight line movement, see first_fail.jpeg
%             goals = [ 2, 0,  2.5, 2.5;
%                      -0.67, 0, 1.67,  -4];
%             goals = [8.5;
%                      -10]; 
            x_g =  goals(:,1); %[16; 10]; % try [20; 5]; [25; 5];
            
            % Obstacle avoidance variables - orbit
            S = 0.36; % Sphere of influence
            R = 0.3; % Radius of orbit
            k_conv = 0.5; % Convergence gain
            
            % Obstacle avoidance variables - barrier
            S_b = 0.27; % Sphere of influence of barrier
            R_b = 0.1; % Radius of full influence
            
            % Weights
            w_g2g = 1;
            w_avoid = 1;
            w_barrier = 10;
            weights = zeros(1+veh.sensor.n_lines*2, 1);
            weights(1) = w_g2g;
            weights(2:1+veh.sensor.n_lines) = w_avoid;
            weights(2+veh.sensor.n_lines:end) = w_barrier;
            
            % Create a weighted field for the vector field to follow
            fields = cell(1+veh.sensor.n_lines*2, 1); % 1 for goal to goal and then the rest for the object avoidance
            avoid_indices = 2:veh.sensor.n_lines+1;
            q_inf = [10000000; 10000000];
            g2g_field_index = 1;
            fields{g2g_field_index} = GoToGoalField(x_vec, y_vec, x_g, v_max);
            fields{g2g_field_index}.setSigma(0.1);
            for k = avoid_indices
                fields{k} = OrbitAvoidField(x_vec, y_vec, q_inf, R, v_max, k_conv, S);  
                fields{k+veh.sensor.n_lines} = AvoidObstacle(x_vec, y_vec, q_inf, v_max);
                fields{k+veh.sensor.n_lines}.S = S_b;
                fields{k+veh.sensor.n_lines}.R = R_b;
            end
            
            % Create a combined vector field
            field = SummedFields(x_vec, y_vec, fields, weights, v_max);
            
            % Create the scenario
            obj = obj@VectorFieldScenario(field, veh, PolygonWorld1, control_type);
            
%             veh.x(veh.q_ind) = [5; -1];
%             veh.x(veh.th_ind) = pi/4;
            
            % Store object variables
            obj.avoid_indices = avoid_indices;
            obj.q_inf = q_inf; 
            obj.n_sensors = veh.sensor.n_lines;
            obj.x_g = x_g;
            obj.goals = goals;
            obj.n_goals = size(goals, 2);
            obj.g2g_field_index = g2g_field_index;
            obj.x_vec = x_vec;
            obj.y_vec = y_vec;
            obj.v_max = v_max;
            obj.tf = 1000; % Set the final time of the simulation
        end
        
        function initializeStatePlot(obj)
            initializeStatePlot@Scenario(obj);
            
            % Initialize the goal plot
            obj.h_goal_plot = plot(obj.plot_ax, obj.x_g(1), obj.x_g(2), 'go', 'linewidth', 3);
        end
        
        function u = control(obj, t, x)
            % Get obstacle avoidance readings into the vector fields
            if ~isempty(obj.vehicle.xo_latest) && ~isempty(obj.vehicle.yo_latest)
                for k = 1:obj.n_sensors
                    q = [obj.vehicle.xo_latest(k); obj.vehicle.yo_latest(k)];
                    if isinf(sum(q))
                        q = obj.q_inf;
                    end
                    obj.vector_field.fields{obj.avoid_indices(k)}.x_o = q;
                    obj.vector_field.fields{obj.avoid_indices(k)+obj.vehicle.sensor.n_lines}.x_o = q;
                end
            else
                warning('No sensor readings yet received');
            end
            
            % Change goal based on distance
            dist = norm(obj.x_g - obj.vehicle.x(obj.vehicle.q_ind));
            if dist < obj.goal_switch
                if obj.n_goals > obj.goal_ind
                    % Update to the next goal
                    obj.goal_ind = obj.goal_ind+1; 
                    obj.x_g = obj.goals(:, obj.goal_ind);
                    
                    % Set the goal within the correct vector field
                    obj.vector_field.fields{obj.g2g_field_index}.x_g = obj.x_g;
                    
                    % Plot the new goal
                    if ~isempty(obj.h_goal_plot)
                        set(obj.h_goal_plot, 'xdata', obj.x_g(1), 'ydata', obj.x_g(2));
                    end
                end            
            end
            
            % Get the control
            u = control@VectorFieldScenario(obj, t, x);
        end
        
        function title_val = getTitle(obj, t)
            dist = norm(obj.x_g - obj.vehicle.x(obj.vehicle.q_ind));
            title_val = ['Sim Time: ' num2str(t, '%.2f') ', Dist = ' num2str(dist, '%.2f') ...
                ', Goal = [' num2str(obj.x_g(1), '%.2f') ', ' num2str(obj.x_g(2), '%.2f'), ']'];
        end
    end
    
    methods (Access=protected)
        function plotWorld(obj, t)
            %obj.vector_field.plotVectorField(t);
        end
    end
end

