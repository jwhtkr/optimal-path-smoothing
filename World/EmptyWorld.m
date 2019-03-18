classdef EmptyWorld < PolygonWorld
    %PolygonWorld1 Is an instantiation of Polygon World with a three polygons
    
    
    methods
        function obj = EmptyWorld()
            % Create the polygon world without any obstacles
            obj = obj@PolygonWorld();
        end
        
        
    end
end

