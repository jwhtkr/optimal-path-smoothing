classdef CombinedGoToGoalVectorOrientationScenario < VectorFieldScenario
    
    properties
        avoid_indices % Stores the indices for the avoidance fields
        q_inf; % Large number to use for placing an obstacle infinitely far away
        n_sensors; % Stores the number of sensors
    end
    
    methods
        function obj = CombinedGoToGoalVectorOrientationScenario(veh, control_type)
            % Plotting variables
            x_vec = -1:1:20;
            y_vec = -6:1:10;
            
            % Vehicle variables
            v_max = 2;
                        
            % Go to goal variables
            x_g = [16; 10];
            
            % Obstacle avoidance variables
            S = 2; % Sphere of influence
            R = 1; % Radius of max influence
            
            % Weights
            w_g2g = 1;
            w_avoid = 1;
            weights = zeros(1+veh.sensor.n_lines, 1);
            weights(1) = w_g2g;
            weights(2:end) = w_avoid;
            
            % Create a weighted field for the vector field to follow
            fields = cell(1+veh.sensor.n_lines); % 1 for goal to goal and then the rest for the object avoidance
            avoid_indices = 2:veh.sensor.n_lines+1;
            q_inf = [10000000; 10000000];
            fields{1} = GoToGoalField(x_vec, y_vec, x_g, v_max);
            for k = avoid_indices
                fields{k} = AvoidObstacleOrientation(x_vec, y_vec, q_inf, v_max);
                fields{k}.S = S;
                fields{k}.R = R;
            end
            
            % Create a combined vector field
            field = SummedFields(x_vec, y_vec, fields, weights, v_max);
            
            % Create the scenario
            obj = obj@VectorFieldScenario(field, veh, PolygonWorld1, control_type);
            
            % Store object variables
            obj.avoid_indices = avoid_indices;
            obj.q_inf = q_inf; 
            obj.n_sensors = veh.sensor.n_lines;
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
            
            % Get the control
            u = control@VectorFieldScenario(obj, t, x);
        end
        
        
        
        
    end
    
    methods (Access=protected)
        function plotWorld(obj, t)
            obj.vector_field.plotVectorField(t);
        end
    end
end

