classdef TwoDRangePlotter < Plotter
    %TwoDRangePlotter Plots a vector field
    
    properties
        vehicle % Instance of Vehicle class
    end
    
    methods
        function obj = TwoDRangePlotter(vehicle)
            %VectorFieldPlotter Construct an instance of this class
            obj = obj@Plotter();
            obj.vehicle = vehicle;
        end
        
        %%%%%%  Plotting functions %%%%%%%%%%%
        function initializePlot(obj, t)
            % Plot the vehicle
            obj.vehicle.sensor.initializePlots(gca);
        end
        
        function plot(obj, t)
            q = obj.vehicle.x(obj.vehicle.q_ind);
            obj.vehicle.sensor.plotMeasurements(q, obj.vehicle.xo_latest, obj.vehicle.yo_latest);
        end
    end
end

