function createOccupancyGrid()
%createOccupancyGrid Creates an occupance grid based on a world file and
%visualizes it
close all

    % Create the world
    world = PolygonWorld1;
    
    % Create an occupance grid
    res = 0.25;
    xlim = [0 22];
    ylim = [-3 10];
    grid = OccupancyGrid(res, xlim, ylim);
    
    % Fill the world with points
    x_vec = xlim(1):res:xlim(2);
    y_vec = ylim(1):res:ylim(2);
    [X, Y] = meshgrid(x_vec, y_vec);
    [n_rows, n_cols] = size(X);
    for row = 1:n_rows
        for col = 1:n_cols
            if world.insideObstacles(X(row, col), Y(row, col))
                grid.setOccupied([X(row, col); Y(row, col)]);
            end
        end
    end
    
    % Plot the world
    figure;
    world.plotWorld(gca);
    
    % Plot the occupancy grid
    figure;
    hold on;
    plotter = OccupancyPlotter(grid);
    plotter.plot_grid = true;
    plotter.initializePlot(0);
    plotter.plot(0);
end

