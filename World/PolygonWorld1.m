classdef PolygonWorld1 < PolygonWorld
    %PolygonWorld1 Is an instantiation of Polygon World with a three polygons
    
    
    methods
        function obj = PolygonWorld1()
            % Define the vertices for various polygons
            V1 = [6, 5, 1; 1, 6, 4];
            V2 = [17, 21, 21, 17; 1, 1, 7, 7];
            V3 = [7, 8, 13, 14, 13, 8; -1.5, -3, -3, -1.5, 0, 0];
            
            % Create the polygon world
            obj = obj@PolygonWorld(V1, V2, V3);
        end
    end
end

