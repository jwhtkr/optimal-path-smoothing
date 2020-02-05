classdef VectorFieldPlotter < Plotter
    %VectorFieldPlotter Plots a vector field
    
    properties
        field % Instance of VectorField to be plotted
    end
    
    methods
        function obj = VectorFieldPlotter(field)
            %VectorFieldPlotter Construct an instance of this class
            obj = obj@Plotter();
            obj.field = field;
        end
        
        %%%%%%  Plotting functions %%%%%%%%%%%
        function initializePlot(obj, t)
            % Plot the vehicle
            obj.field.plotVectorField(t);
        end
        
        function plot(obj, t)
            obj.field.plotVectorField(t);
        end
    end
end

