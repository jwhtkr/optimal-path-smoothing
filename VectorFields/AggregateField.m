classdef AggregateField < VectorField
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fields % An aggregate set of fields
        n_fields % Number of fields aggregated
    end
    
    methods
        function obj = AggregateField(x_vec, y_vec)
             %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            
            % Create a variety of fields
            r = 4;
            %obj.fields{1} = OrbitField(x_vec, y_vec, [-5; -5], r, 1);
            obj.fields{1} = OrbitField(x_vec, y_vec, [-5; 5], r, 1);
            obj.fields{2} = OrbitField(x_vec, y_vec, [5; -5], r, -1);
            %obj.fields{4} = OrbitField(x_vec, y_vec, [5; 5], r, 1);
            
            obj.n_fields = length(obj.fields);
        end
        
        function g = getVector(obj, t, x)
            g = [0;0];
            
            for k = 1:obj.n_fields
                g = g + obj.getWeight(x, k)*obj.fields{k}.getVector(t,x);
            end
        end
        
        function w = getWeight(obj, x, k)
           d = norm(x - obj.fields{k}.x_c);
           w = exp(-d);
        end
    end
end

