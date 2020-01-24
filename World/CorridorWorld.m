classdef CorridorWorld < PolygonWorld
    %PolygonWorld1 Is an instantiation of Polygon World with a three polygons
    
    
    methods
        function obj = CorridorWorld()
            % Define the vertices for various polygons
            V1 = [-2, 10, 10, 18, 18, -2; 1.75, 1.75, 10, 10, 12, 12];
%             V1 = [-2, 10, 11, 11.25, 10, 18, 18, -2; 1.75, 1.75, 1, 1.5, 10, 10, 12, 12];
            V2 = [-2, 14, 14, 18, 18, -2;-1.75, -1.75, 6, 6, -4, -4];
            
%             V3 = [3,3.25, 3.5, 3.25;0,.25, 0, -.25];
            
            
            % Create the polygon world
%             obj = obj@PolygonWorld(V1, V2, V3);
            obj = obj@PolygonWorld(V1, V2);
        end
        
        
    end
end

