classdef VectorField < handle
    %VectorField is an abstract class which gives a basic vector field for
    %navigation
    
    
    properties (SetAccess = protected, GetAccess = public)
        % Plotting variables
        X_grid  % Grid Variables used for plotting
        Y_grid  
        quiver_handle = [] % Handles to arrows plotted
        plot_with_unit_vector = false; % true => use the unit vector to plot
    end
    
    methods (Abstract)
        g = getVector(obj, t, x, th);
    end
    
    methods
        function obj = VectorField(x_vec, y_vec)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            
            % Create the meshgrid from the data inputs
            [obj.X_grid, obj.Y_grid] = meshgrid(x_vec, y_vec);
        end
        
        function h = plotVectorField(obj, t)
            % Initialize the velocity values
            [m,n] = size(obj.X_grid);
            U = zeros(m,n);
            V = zeros(m,n);
            
            % Loop through and update all of the arrows
            for r = 1:m
                for c = 1:n
                    % Get the corresponding state
                    x = [obj.X_grid(r,c); obj.Y_grid(r,c)];
                    
                    % Get the vector
                    if obj.plot_with_unit_vector
                        g = obj.getUnitVector(t, x, 0);
                    else
                        g = obj.getVector(t, x, 0);
                    end
                    
                    % Store the vector
                    U(r,c) = g(1);
                    V(r,c) = g(2);
                end
            end
            
            % Plot the quiver
            if isempty(obj.quiver_handle)
                obj.quiver_handle = quiver(obj.X_grid, obj.Y_grid, U, V, 'g');
            else
                set(obj.quiver_handle, 'Udata', U, 'Vdata', V);
            end
            
            % Return the handle to the quiver group
            h = obj.quiver_handle;
        end
        
        function g = getUnitVector(obj, t, x, th)
            g = obj.getVector(t, x, th);
            g = g ./ norm(g);
        end
    end
end

