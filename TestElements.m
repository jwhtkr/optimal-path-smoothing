function TestElements()
    % Clear previous work
    close all;
    clear;
    clc;
    
    % Configure file path
    addpath Dependencies\intersections
    addpath Scenario
    addpath Sensors
    addpath Vehicle
    addpath Vehicle\Kinematics
    addpath Vehicle\LowLevelControl
    addpath World

    % Create a temporary figure;
    figure;
    ax = gca;
    
    % Create a world 
    world = PolygonWorld();
    world.plotWorld(ax);
    axis equal;
    
    % Create a range sensor
    range = RangeSensor();
    range.initializePlots(ax);
    
    % Loop through and set random points
    x = rand * 10;
    y = rand * 10;
    h_pnt = plot(ax, x, y, 'bo', 'linewidth', 4);
    for k = 1:10
        % Create the point of interest
        x = rand * 10;
        y = rand * 10;    
        th = rand * 2*pi;
        set(h_pnt, 'xdata', x, 'ydata', y);
        
        % Update the range data
        q = [x; y];
        [xo, yo, dist_o] = range.getObstacleDetections(q, th, world);
        
        % Plot the measurements
        range.plotMeasurements(q, xo, yo);
    end

end

