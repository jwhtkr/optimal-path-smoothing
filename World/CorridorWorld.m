classdef CorridorWorld < PolygonWorld
    %PolygonWorld1 Is an instantiation of Polygon World with a three polygons
    
    
    methods
        function obj = CorridorWorld()
            % Define the vertices for various polygons
            V1 = [-2, 10, 10, 18, 18, -2; 1.75, 1.75, 10, 10, 12, 12];
%             % 4 agents
%             V1 = [-2, 4, 9, 11, 18, 18, -2; 1.75, .9, .9, 10, 10, 12, 12];
%             V2 = [-2, 4, 14, 14, 18, 18, -2;-1.75, -.9, -.9, 6, 6, -4, -4];
%             V3 = [11.7 12.4 12.4 11.7; 7.4 7.4 8.1 8.1];
            % 3 agents
            V1 = [-2, 4, 10, 11, 18, 18, -2; 1.75, .75, .75, 10, 10, 12, 12];
            V2 = [-2, 4, 14, 14, 18, 18, -2;-1.75, -.75, -.75, 6, 6, -4, -4];
            V3 = [12.2 12.8 12.8 12.2; 7.2 7.2 7.8 7.8];
            
%             V3 = [3,3.25, 3.5, 3.25;0,.25, 0, -.25];
            

            
            
            % Create the polygon world
            obj = obj@PolygonWorld(V1, V2, V3);
%             obj = obj@PolygonWorld(V1, V2);
        end
        
        
    end
end

