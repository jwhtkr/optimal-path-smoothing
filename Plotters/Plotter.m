classdef Plotter < handle
    %Plotter plots a component of the simulation
    
    methods(Abstract)
        initializePlot(obj, t);
        plot(obj, t)
    end
    
    methods
        function obj = Plotter()            
        end
    end
end

