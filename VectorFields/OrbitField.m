classdef OrbitField < VectorField
    %OrbitField Creates a vector field moving the vehicle into orbit
    
    properties
        x_c % 2D center position of the orbit
        rad % radius of the orbit
        w % frequency of the orbit
        k_conv % gain on convergence to the orbit
        v_d % Desired speed of the orbit (calculated using w*rad)
    end
    
    methods
        function obj = OrbitField(x_vec, y_vec, x_c, rad, w, k_conv)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   x_c - Defines a 2D center position for the vector field
            %   rad - radius of the orbit
            %   w - frequency of orbit (positive produces clockwise motion,
            %   negative produces counterclockwise)
            %   k_conv - gain on convergence to the orbit
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            
            % Store the variables which describe the orbit
            obj.x_c = x_c;
            obj.rad = rad;
            obj.w = w;
            obj.k_conv = k_conv;
            obj.v_d = abs(obj.rad*obj.w); % Note: this should be our forward velocity
        end
        
        function g = getVector(obj, t, x, th)
            xhat = x - obj.x_c;
            gam = obj.k_conv*(obj.rad^2 - (xhat'*xhat));            
            
            A = [gam, obj.w; -obj.w, gam];
            g = A*xhat;   
            
            % Scale the vector field
            v_g = norm(g);
            if v_g > obj.v_d
               g = obj.v_d/v_g * g; 
            end
        end
        
        function h = plotVectorField(obj, t)
            % Plot the circular radius
            circle(obj.x_c, obj.rad, 20, 'b', []);
            
            % Create the plot of the actual vector field
            h = plotVectorField@VectorField(obj, t);
        end
    end
end

