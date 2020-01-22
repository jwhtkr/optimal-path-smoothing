classdef ZeroControl < Controller
    %ZeroControl outputs zeros for the control
    
    properties
        u_out % Number of inputs
    end
    
    methods
        function obj = ZeroControl(vehicle,n_u)
            obj = obj@Controller(vehicle);
            obj.u_out = zeros(n_u, 1);
        end
        
        function u = calculateControl(t, x)
            u = obj.u_out;
        end
    end
end

