classdef Circle < handle

    properties
        x;
        y;
        r;
    end
    
    methods
        function obj = Circle(x,y,r)
            obj.x = x;
            obj.y = y;
            obj.r = r;
        end
    end
end
        