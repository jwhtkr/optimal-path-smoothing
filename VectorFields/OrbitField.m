classdef OrbitField < VectorField
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x_c % 2D center position of the orbit
        rad % radius of the orbit
        w % frequency of the orbit
    end
    
    methods
        function obj = OrbitField(x_vec, y_vec, x_c, rad, w)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   x_c - Defines a 2D center position for the vector field
            %   rad - radius of the orbit
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            obj.x_c = x_c;
            obj.rad = rad;
            obj.w = w;
        end
        
        function g = getVector(obj, t, x)
            xhat = x - obj.x_c;
            gam = .01*(obj.rad^2 - (xhat'*xhat));
            
            A = [gam, obj.w; -obj.w, gam];
            g = A*xhat;            
        end
    end
end

