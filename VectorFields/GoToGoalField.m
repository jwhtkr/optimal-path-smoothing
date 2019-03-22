classdef GoToGoalField < VectorField
    %GoToGoalField Basic vector field pointing to a desired goal point
    
    properties (SetAccess = public, GetAccess = public)
        x_g % Goal position
        v_max % Max velocity
    end
    
    methods
        function obj = GoToGoalField(x_vec, y_vec, x_g, v_max)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   x_g - Defines a 2D goal position for the vector field
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            obj.x_g = x_g;
            obj.v_max = v_max;
        end
        
        function g = getVector(obj, t, x)
            g = obj.x_g - x;
            
            % Scale the magnitude of the resulting vector
            v_g = norm(g);
            if v_g > obj.v_max
                g = obj.v_max/v_g * g;
            end
        end
    end
end

