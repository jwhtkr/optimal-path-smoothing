classdef SwithingLineScenarioObstacleAvoidBetterSwitch < VectorFieldScenario
    %SwithingLineScenario Uses a simple distance based switching function
    %to switch between lines
    
    properties
        avoid_indices % Stores the indices for the avoidance fields
        q_inf; % Large number to use for placing an obstacle infinitely far away
        n_sensors; % Stores the number of sensors
        
        x_g % Series of goal's which then define the lines
        n_goals % Number of goals (number of columns in x_g)
        current_goal = 2 % The column in x_g where the robot is headed
        
        % Half plan variables
        a = 0;      % Definition of a line: f = a*x + b*y + c;
        b = 0;
        c = 0;
    end
    
    methods
        function obj = SwithingLineScenarioObstacleAvoidBetterSwitch(veh, control_type)
            % Plotting variables
            x_vec = -1:1:20;
            y_vec = -6:1:10;
            
            % Vehicle variables
            v_max = 2;
                        
            % Intermediate goal variables
            x_g = [0, 0; 6, 0; 11, 6; 19, -4; 22, 4; 16, 10]';
            n_goals = size(x_g, 2);
            
            % Obstacle avoidance variables
            S = 3; % Sphere of influence
            R = 2; % Radius of orbit
            k_conv = 0.5; % Convergence gain
            
            % Line variables
            x_l = [0;0]; % Dummy variable - the goal is updated after initialization
            psi_l = 0; % Dummy variable - the orientation is updated after orientation
            slope = 1;
            
            % Weights
            w_line = 1;
            w_avoid = 1;
            weights = zeros(1+veh.sensor.n_lines, 1);
            weights(1) = w_line;
            weights(2:end) = w_avoid;
            
            % Create a weighted field for the vector field to follow
            fields = cell(1+veh.sensor.n_lines); % 1 for goal to goal and then the rest for the object avoidance
            avoid_indices = 2:veh.sensor.n_lines+1;
            q_inf = [10000000; 10000000];
            fields{1} = LineVectorField(x_vec, y_vec, x_l, psi_l, slope, v_max);
            for k = avoid_indices
                fields{k} = OrbitAvoidField(x_vec, y_vec, q_inf, R, v_max, k_conv, S);                
            end
            
            % Create a combined vector field
            field = SummedFields(x_vec, y_vec, fields, weights, v_max);
            
            % Create the scenario
            obj = obj@VectorFieldScenario(field, veh, PolygonWorld1, control_type);
            obj.tf = 100000; % basically never end
            
            % Store object variables
            obj.avoid_indices = avoid_indices;
            obj.q_inf = q_inf; 
            obj.n_sensors = veh.sensor.n_lines;
            obj.x_g = x_g;
            obj.n_goals = n_goals;
            
            % Update the goal to create the half plane
            obj.updateGoal(obj.current_goal); % This will create the goal point, orientation, and half plane
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
                end
            else
                warning('No sensor readings yet received');
            end
            
            % Evaluate switching condition and check for new goal if needed
            new_goal_index = obj.switchingFunction(t, x);
            if new_goal_index ~= obj.current_goal
                obj.updateGoal(new_goal_index);
            end            
            
            % Get the control
            u = control@VectorFieldScenario(obj, t, x);
        end
        
        function new_goal = switchingFunction(obj, t, x)
            % Get the current position and goal
            q = x(obj.q_ind);   % position
            
            % Evaluate the half plane
            f = obj.evalHalfplane(q(1), q(2));
            
            % Update the new goal
            new_goal = obj.current_goal;
            if f < 0 && obj.current_goal < obj.n_goals
                new_goal = obj.current_goal + 1;
            end
        end
        
        function updateGoal(obj, new_goal_index)
            % Update the goal index
            new_goal_index
            obj.current_goal = new_goal_index;
            
            % Get the new points on the line
            p1 = obj.x_g(:, obj.current_goal-1);
            p2 = obj.x_g(:, obj.current_goal);
            
            % Update the line function
            [x_l, psi_l] = SwithingLineScenario.getLineVariables(p1, p2);
            obj.vector_field.fields{1}.setLineParameters(x_l, psi_l);
            
            % Update the half plane
            obj.updateHalfPlane(p2, psi_l);
        end
        
        %%%%%%%%%%%%%%%  Half plane functions %%%%%%%%%%%%%%%%%%%%%%%%
        function updateHalfPlane(obj, goal, psi)
            % Create half plane verticies
            v1 = goal;
            psi_orth = psi - pi/2;
            v2 = v1 + [cos(psi_orth); sin(psi_orth)];
            
            % Create the half plane values
            [obj.a, obj.b, obj.c] = obj.createHalfPlane(v1, v2);
        end
        
        function [a, b, c] = createHalfPlane(obj, v1, v2)
            % Extract values
            x1 = v1(1);
            y1 = v1(2);
            x2 = v2(1);
            y2 = v2(2);
            
            % Calculate a and b
            a = y2 - y1;
            b = x1 - x2;
            
            % Calculate c
            c = -a*x1 - b*y1;
        end
        
        
        function f = evalHalfplane(obj, x, y)
            f = obj.a*x + obj.b*y + obj.c;
        end
        
    end
    
    methods (Access=protected)
        function plotWorld(obj, t)
            obj.vector_field.plotVectorField(t);
        end
    end
    
    methods (Static)
        function [x_l, psi_l] = getLineVariables(p1, p2)
           v = p2-p1;
           psi_l = atan2(v(2), v(1));
           x_l = p2;
        end
    end
end

