classdef OccupancyPlotter < Plotter
    %OccupancyPlotter Performs a simple plotting of the occupance grid
    
    properties
        grid_plot % Grid that is used for comparison with the OccupancyGrid
        sz_grid % size of the grid_plot
        grid_handle % Instance of OccupancyGrid that is used by the vehicle to plot
        
        % Plotting properties
        ax_handle = [] % Handle to the axis for plotting the occupancy grid
        plot_grid = false; % True => plot the grid
    end
    
    methods
        function obj = OccupancyPlotter(grid)
            obj.grid_plot = uint8(zeros(size(grid.grid)));
            obj.sz_grid = size(obj.grid_plot);
            obj.grid_handle = grid;
        end
        
        function initializePlot(obj, t)
            if ~isempty(obj.ax_handle)
                return;
            end
            
            % Set properties for axis
            obj.ax_handle = gca; hold on;
            set(obj.ax_handle, 'xlim', obj.grid_handle.x_lim);
            set(obj.ax_handle, 'ylim', obj.grid_handle.y_lim);
            axis equal;
            
            % Plot the grid
            if obj.plot_grid
                obj.plotGrid();
            end
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
        
        function plotGrid(obj)
            % Define the color
            color = 0.5.*[1, 1, 1]; % Have the grid lines be grey
            
            % Get the spacing values for the grid
            x_vec = obj.grid_handle.x_lim(1):obj.grid_handle.res:obj.grid_handle.x_lim(2);
            y_vec = obj.grid_handle.y_lim(1):obj.grid_handle.res:obj.grid_handle.y_lim(2);
            
            % Loop through each y value and plot the horizontal grids
            for y = y_vec
                plot(obj.ax_handle, obj.grid_handle.x_lim, [y y], 'color', color);
            end
            
            % Loop through each x value and plot the vertical lines of the
            % grid
            for x = x_vec
                plot(obj.ax_handle, [x x], obj.grid_handle.y_lim,  'color', color);
            end
        end
    end
end

