classdef TrajectoryPlotter < Plotter
    %TrajectoryPlotter Plots a trajectory as a given color
    
    properties
        trajectoryFunction % handle to a function to get a desired trajectory
        color % 1x3 color vector indicating the color to plot
        h_pos = [] % handle to the position plot
    end
    
    methods
        function obj = TrajectoryPlotter(trajectoryFunction, color)
            obj.trajectoryFunction = trajectoryFunction;
            obj.color = color;
        end
        
        function initializePlot(obj, t)
            % Get the desired position
            q_mat = obj.trajectoryFunction(t);
            
            % Plot the desired position
            obj.h_pos = plot(q_mat(1,:), q_mat(2, :), 'color', obj.color, 'linewidth', 2);
        end
        
        function plot(obj, t)
            % Get the desired position
            q_mat = obj.trajectoryFunction(t);
            
            % Update the position
            set(obj.h_pos, 'xdata', q_mat(1,:), 'ydata', q_mat(2,:));
        end
    end
end

