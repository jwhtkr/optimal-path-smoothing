classdef SingleAgent < handle
    %SingleAgent Implements a control for a vehicle
    
    properties
        vehicle % Instance of teh Vehicle class
        world % Instance of the Polygon world class
    end
    
    methods(Abstract)
        u = control(obj, t, x, agents) % Calculate the control given the time and state        
    end
    
    methods
        function obj = SingleAgent(vehicle,world)
            obj.vehicle = vehicle;
            obj.world = world;
        end
        
        function xdot = dynamics(obj, t, x, u)
            xdot = obj.vehicle.kinematics.kinematics(t, x, u);
        end
        
        function processSensors(obj, t, x, agents)
            % Updates any sensor values
            obj.vehicle.getObstacleDetections(obj.world);
        end
    end
end

