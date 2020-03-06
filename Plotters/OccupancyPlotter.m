classdef OccupancyPlotter < Plotter
    %OccupancyPlotter Performs a simple plotting of the occupance grid
    
    properties
        grid_plot % Grid that is used for comparison with the OccupancyGrid
        sz_grid % size of the grid_plot
        grid_handle % Instance of OccupancyGrid that is used by the vehicle to plot
        
        % Plotting properties
        ax_handle = [] % Handle to the axis for plotting the occupancy grid        
        x_lim % x limits
        y_lim % y limits
    end
    
    methods
        function obj = OccupancyPlotter(grid)
            obj.grid_plot = uint8(zeros(size(grid.grid)));
            obj.sz_grid = size(obj.grid_plot);
            obj.grid_handle = grid;
            
            % Store plotting limits
            obj.x_lim = grid.x_lim;
            obj.y_lim = grid.y_lim;
        end
        
        function initializePlot(obj, t)
            if ~isempty(obj.ax_handle)
                return;
            end
            
            % Set properties for axis
            obj.ax_handle = gca; hold on;
            set(obj.ax_handle, 'xlim', obj.x_lim);
            set(obj.ax_handle, 'ylim', obj.y_lim);
            axis equal;
            
            
            obj.plot(t);
        end
        
        function plot(obj, ~)
            % Don't do anything if the plot is not initialzied
            if isempty(obj.ax_handle)
                return;
            end
            
            % Get the difference in grid values from previous plotting
            plot_grid = obj.grid_handle.grid - obj.grid_plot;
            ind = find(plot_grid ~= 0);
            
            % Update the plot grid to the new grid values
            obj.grid_plot(ind) = obj.grid_handle.grid(ind);
            
            % Get the position to plot
            [row, col] = ind2sub(obj.sz_grid, ind);
            len = length(row);
            q_mat = zeros(2, len);
            for k = 1:len
                q_mat(:,k) = obj.grid_handle.indexToPosition(row(k), col(k));
            end
            
            % Plot the position
            if len > 0
                plot(obj.ax_handle, q_mat(1,:), q_mat(2,:), 'ko');            
            end
        end
    end
end

