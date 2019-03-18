classdef BetterUnicycleVehicle < Vehicle
   
    properties
        Property1
    end
    
    methods
        function obj = BetterUnicycleVehicle(x0)
            kin = BetterUnicycle;
            q_ind = [kin.x_ind; kin_y_ind];
            obj = obj@Vehicle(kin, x0, q_ind);            
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

