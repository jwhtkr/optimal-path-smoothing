classdef AvoidObstacleOrientation < VectorField
    %GoToGoalField Basic vector field pointing to a desired goal point
    
    properties (SetAccess = public, GetAccess = public)
        x_o % Obsactle position
        v_max % Max velocity
        
        % Convergence variables
        S = 5 % Sphere of influence
        R = 1 % Radius of max effect    
        
        % Orientation variables
        So = 3*pi/4;
        Ro = pi/4;
    end
    
    methods
        function obj = AvoidObstacleOrientation(x_vec, y_vec, x_o, v_max)
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
            
            % Scale the magnitude of the resulting vector (Sphere of
            % influence)
            dist = norm(g);
            scale_sphere = 1;
            if dist > obj.S
                scale_sphere = 0;
            elseif dist > obj.R
                scale_sphere = (obj.S - dist) / (obj.S - obj.R);
            end
            
            % Scale the magunitude of the resulting vector by the error
            % orientation
            th_g = atan2(-g(2), -g(1)); % -g as we want to know the heading to the obstacle
            th_e = th-th_g;
            th_e = abs(atan2(sin(th_e), cos(th_e))); % Error in orientation from pointing to the obstacle
            scale_orien = 1; % Scaling due to orientation difference
            if th_e > obj.So
                scale_orien = 0;
            elseif dist > obj.R
                scale_orien = (obj.So - th_e) / (obj.So - obj.Ro);            
            end
            
            % Create the velocity scaled by the influence factors
            v_g = obj.v_max * scale_sphere * scale_orien;
            
            % Output g
            if dist > 0 % Avoid dividing by zero
                g = v_g/dist * g; % Dividing by dist is dividing by the norm
            else % Choose a random position if you are on top of the obstacle
                g = rand(2,1);
            end
        end
    end
end

