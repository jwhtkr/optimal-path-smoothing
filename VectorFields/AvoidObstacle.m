classdef AvoidObstacle < VectorField
    %GoToGoalField Basic vector field pointing to a desired goal point
    
    properties (SetAccess = public, GetAccess = public)
        x_o % Goal position
        v_max % Max velocity
        
        % Convergence variables
        S = 5 % Sphere of influence
        R = 1 % Radius of max effect        
    end
    
    methods
        function obj = AvoidObstacle(x_vec, y_vec, x_o, v_max)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   x_g - Defines a 2D goal position for the vector field
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            obj.x_o = x_o;
            obj.v_max = v_max;
        end
        
        function g = getVector(obj, t, x, th)
            g = x - obj.x_o;
            
            % Scale the magnitude of the resulting vector
            dist = norm(g);
            scale = 1;
            if dist > obj.S
                scale = 0;
            elseif dist > obj.R
                scale = (obj.S - dist) / (obj.S - obj.R);
            end
            v_g = obj.v_max * scale;
            
            % Output g
            if dist > 0 % Avoid dividing by zero
                g = v_g/dist * g; % Dividing by dist is dividing by the norm
            else % Choose a random position if you are on top of the obstacle
                g = rand(2,1);
            end
        end
    end
end

