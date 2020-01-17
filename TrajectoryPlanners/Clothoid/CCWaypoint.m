classdef CCWaypoint < handle
    properties
        x;
        y;
        psi;
        C_l;
        C_l_reverse;
        C_l_inner;
        C_l_reverse_inner;
        
        C_r;
        C_r_reverse;
        C_r_inner;
        C_r_reverse_inner;
        mu;
    end
    
    methods
        function obj = CCWaypoint(waypoint)
            obj.x = waypoint(1);
            obj.y = waypoint(2);
            obj.psi = waypoint(3);
            
            
            obj.C_l = Circle(0, 0, 0);
            obj.C_l_reverse = Circle(0, 0, 0);
            obj.C_l_inner = Circle(0, 0, 0);
            obj.C_l_reverse_inner = Circle(0, 0, 0);
            
            obj.C_r = Circle(0, 0, 0);
            obj.C_r_inner = Circle(0, 0, 0);
            obj.C_r_reverse = Circle(0, 0, 0);
            obj.C_r_reverse_inner = Circle(0, 0, 0);
        end
    end
end