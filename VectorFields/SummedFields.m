classdef SummedFields < VectorField
    %SummedFields sums together multiple fields to get the contribution
    
    properties
        fields % A cell structure where each element is an instance of the VectorField object
        n_fields % Length of fields
        weights % A vector of dimension n_fields which provides a weight for each field
        v_max % Maximum velocity produced by the field
    end
    
    methods
        function obj = SummedFields(x_vec, y_vec, fields, weights, v_max)
            % Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   fields % A cell structure where each element is an instance of the VectorField object
            %   weights % A vector of dimension n_fields which provides a weight for each field 
            %   v_max % Maximum velocity produced by the field
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            
            % Store the variables
            obj.fields = fields;
            obj.n_fields = length(fields);
            obj.weights = weights;
            obj.v_max = v_max;
        end
        
        function g = getVector(obj, t, x, th)
            g = zeros(2,1);
            for k = 1:obj.n_fields
                g = g + obj.weights(k)*obj.fields{k}.getVector(t, x, th);
            end    
            
            % Saturate the field to have a maximum velocity of v_max
            v_g = norm(g);
            if v_g > obj.v_max
                g = obj.v_max/v_g * g;
            end
        end
    end
end

