classdef Controller < handle
    %Controller Abstract class for calculating the control for a vehicle
    
    properties(SetAccess=protected)
        vehicle % Instance of the UncontrolledVehicle class
    end
    
    methods
        function obj = Controller(vehicle)
            obj.vehicle = vehicle;
        end        
    end
    
    methods(Abstract)
        u = calculateControl(t, x);
    end
end

