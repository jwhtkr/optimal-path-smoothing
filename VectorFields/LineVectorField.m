classdef LineVectorField < VectorField
    %LineVectorField Vector field causing vehicle to move to a line
    
    properties (SetAccess = public, GetAccess = public)
        % Line variables
        x_l % A point on the line
        psi_l % The orientation of the line
        R_lc % The rotation matrix to rotate a cartesian point to line frame
        R_cl % The rotation matrix to rotate a line point to the cartesian frame
        
        % Vector field variables
        slope % Slope of the sigmoid function defining the line
        v_d % Desired velocity / length of the vector field line
        
    end
    
    methods
        function obj = LineVectorField(x_vec, y_vec, x_l, psi_l, slope, v_d)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   x_g - Defines a 2D goal position for the vector field
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            
            % Store the line specific variables
            obj.x_l = x_l;
            obj.psi_l = psi_l;
            obj.slope = slope;
            obj.v_d = v_d;
            
            % Calculate the rotation matrix to rotate cartesian 
            c = cos(obj.psi_l);
            s = sin(obj.psi_l);
            obj.R_cl = [c -s; s c];
            obj.R_lc = obj.R_cl';
        end
        
        function g = getVector(obj, t, x, th)
            % Calculate the distance to the line
            d = [0 1]*obj.R_lc*(x-obj.x_l);
            
            % Calculate the desired angle of the vector in the line
            % coordinate frame
            psi_d = pi/2*(1-2/(1+exp(-obj.slope*d)));
            
            % Calcualte the unit vector
            vec = [cos(psi_d); sin(psi_d)];
            
            % Calculate the desired vector
            g = obj.v_d*obj.R_cl*vec;            
        end
    end
end