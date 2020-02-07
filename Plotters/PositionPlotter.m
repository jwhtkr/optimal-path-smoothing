classdef PositionPlotter < Plotter
    %PositionPlotter Plots a position as a circle
    
    properties
        positionFunction % handle to a function to get a desired position
        color % 1x3 color vector indicating the color to plot
        h_pos = [] % handle to the position plot
    end
    
    methods
        function obj = PositionPlotter(positionFunction, color)
            obj.positionFunction = positionFunction;
            obj.color = color;
        end
        
        function initializePlot(obj, t)
            % Get the desired position
            qd = obj.positionFunction(t);
            
            % Plot the desired position
            obj.h_pos = plot(qd(1), qd(2), 'o', 'color', obj.color, 'linewidth', 2);
        end
        
        function plot(obj, t)
            % Get the desired position
            qd = obj.positionFunction(t);
            
            % Update the position
            set(obj.h_pos, 'xdata', qd(1), 'ydata', qd(2));
        end
    end
end

