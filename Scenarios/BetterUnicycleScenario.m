classdef BetterUnicycleScenario < Scenario
    %BetterUnicycleScenario This scenario uses the better unicycle robot
        
    properties
        
    end

    methods (Abstract)
        u = control(obj, t, x);
    end
    
    methods
        function obj = BetterUnicycleScenario()
            % Create the unicycle vehicle
            x0 = [0; 0; 0; 0; 0]; % Initial state
            veh = BetterUnicycleVehicle(x0);
            
            % Create the world
            %world = EmptyWorld();
            world = PolygonWorld1();
            
            % initialize the scenario
            obj = obj@Scenario(veh, world, true);            
        end
    end
    
    
end



