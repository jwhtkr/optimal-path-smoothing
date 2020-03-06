classdef SingleAgent < handle
    %SingleAgent Implements a control for a vehicle
    
    properties
        vehicle % Instance of teh Vehicle class
        world % Instance of the Polygon world class
        map % Instance of OccupancyGrid, populated with range measurements        
    end
    
    properties(Constant)
        map_res = 0.1;
    end
    
    methods(Abstract)
        u = control(obj, t, x, agents) % Calculate the control given the time and state        
    end
    
    methods
        function obj = SingleAgent(vehicle,world)
            obj.vehicle = vehicle;
            obj.world = world;
            obj.map = OccupancyGrid(obj.map_res, obj.world.x_lim, obj.world.y_lim);
        end
        
        function xdot = dynamics(obj, t, x, u)
            xdot = obj.vehicle.kinematics.kinematics(t, x, u);
        end
        
        function processSensors(obj, t, x, agents)
            % Updates any sensor values
            obj.vehicle.getObstacleDetections(obj.world);
            
            % Store sensor data into map
            for k = 1:length(obj.vehicle.xo_latest)
                q = [obj.vehicle.xo_latest(k); obj.vehicle.yo_latest(k)];
                obj.map.setOccupied(q);
            end
        end
    end
end

