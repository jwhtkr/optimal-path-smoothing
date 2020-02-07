classdef SingleAgentPlotter < Plotter
    %SingleAgentPlotter plots a single agent
    
    properties
        vehicle % Instance of teh Vehicle class   
        color % 1x3 color vector
    end
    
    methods
        function obj = SingleAgentPlotter(vehicle, color)
            obj = obj@Plotter();
            obj.vehicle = vehicle;
        end
        
        %%%%%%  Plotting functions %%%%%%%%%%%
        function initializePlot(obj, t)
            % Plot the vehicle
            obj.vehicle.initializePlots(gca);
        end
        
        function plot(obj, t)
            % Plot the vehicle
            obj.vehicle.plotVehicle();
        end
    end
end

