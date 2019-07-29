classdef GoToGoalField < VectorField
    %GoToGoalField Basic vector field pointing to a desired goal point
    
    properties (SetAccess = public, GetAccess = public)
        x_g % Goal position
        v_max % Max velocity
        
    end
    
    properties(SetAccess = protected, GetAccess = public)
        % Convergence variables
        sig = 1 % Effects the convergence to zero velocity through 1-exp(-d^2/sig^2)
        sig_sq % Sig^2
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
            
            % Initialize scaling variable
            obj.sig_sq = obj.sig^2;
        end
        
        function g = getVector(obj, t, x, th)
            g = obj.x_g - x;
            
            % Scale the magnitude of the resulting vector
            dist = norm(g);
            v_g = obj.v_max * (1- exp(-dist^2/obj.sig_sq));
            
            if dist > 0 % Avoid dividing by zero
                g = v_g/dist * g; % Dividing by dist is dividing by the norm
            else
                g = [0;0];
            end
        end
        
        function setSigma(obj, sig_in)
            obj.sig = sig_in;
            obj.sig_sq = obj.sig^2;
        end
    end
end

