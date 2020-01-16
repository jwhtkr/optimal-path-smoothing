classdef AvoidObstacleConst < VectorField
    %AvoidObstacleConst Basic vector field pointing away from an obstacle
    %point with a given magnitude
    
    properties (SetAccess = public, GetAccess = public)
        x_o % Obsactle position
        g_mag % Magnitude of vector
        
        S % Sphere of influence
    end
    
    methods
        function obj = AvoidObstacleConst(x_vec, y_vec, x_o, g_mag, S)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   x_g - Defines a 2D goal position for the vector field
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            obj.x_o = x_o;
            obj.g_mag = g_mag;
            obj.S = S;
        end
        
        function g = getVector(obj, t, x, th)
            g = x - obj.x_o;
            
            % Scale the magnitude of the resulting vector
            dist = norm(g);
            if dist > obj.S
                g = [0;0];
            else
                g = obj.g_mag/dist .* g;            
            end
        end
    end
end

