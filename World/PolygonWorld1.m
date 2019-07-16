classdef PolygonWorld1 < PolygonWorld
    %PolygonWorld1 Is an instantiation of Polygon World with a three polygons
    
    
    methods
        function obj = PolygonWorld1()
%             % Define the vertices for various polygons
%             V1 = [6, 5, 1; 1, 6, 4];
%             V2 = [17, 21, 21, 17; 1, 1, 7, 7];
%             V3 = [7, 8, 13, 14, 13, 8; -1.5, -3, -3, -1.5, 0, 0];
%             
%             % Create the polygon world
%             obj = obj@PolygonWorld(V1, V2, V3);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % for the scenario (set as a bunch of touching rectangles, will try for complex objects later)
            % OBJ_i = [ x_1, x_2, ... x_n;   y_1, y_2, ... y_n ];
            % 1/3 sized
            n = 2;
            OBJ1  = [ -1/n,  2/n,  2/n, -1/n;   -1/n, -1/n,-15/n,-15/n ];
            OBJ2  = [  2/n, 31/n, 31/n,  2/n;  -14/n,-14/n,-15/n,-15/n ];
            OBJ3  = [  2/n,  4/n,  4/n,  2/n;   -2/n, -2/n, -7/n, -7/n ];
            OBJ4  = [  4/n,  7/n,  7/n,  4/n;    2/n,  2/n, -7/n, -7/n ];
            OBJ5  = [  4/n,  7/n,  7/n,  4/n;   -9/n, -9/n,-12/n,-12/n ];
            OBJ6  = [ -1/n,  2/n,  2/n, -1/n;    5/n,  5/n,  1/n,  1/n ];
            OBJ7  = [  2/n, 31/n, 31/n,  2/n;    5/n,  5/n,  4/n,  4/n ];
            OBJ8  = [  9/n, 13/n, 13/n,  9/n;  -13/n,-13/n,-14/n,-14/n ];
            OBJ9  = [ 15/n, 21/n, 21/n, 15/n;  -13/n,-13/n,-14/n,-14/n ];
            OBJ10 = [ 23/n, 27/n, 27/n, 23/n;  -13/n,-13/n,-14/n,-14/n ];
            OBJ11 = [  9/n, 10/n, 10/n,  9/n;   -3/n, -3/n,-11/n,-11/n ];
            OBJ12 = [  9/n, 12/n, 12/n,  9/n;    2/n,  2/n, -3/n, -3/n ];
            OBJ13 = [ 12/n, 16/n, 16/n, 12/n;    2/n,  2/n,    0,    0 ];
            OBJ14 = [ 14/n, 16/n, 16/n, 14/n;      0,    0, -5/n, -5/n ];
            OBJ15 = [ 12/n, 16/n, 16/n, 12/n;   -5/n, -5/n,-11/n,-11/n ];
            OBJ16 = [ 18/n, 20/n, 20/n, 18/n;    2/n,  2/n,    0,    0 ];
            OBJ17 = [ 20/n, 27/n, 27/n, 20/n;    2/n,  2/n, -2/n, -2/n ];
            OBJ18 = [ 20/n, 23/n, 23/n, 20/n;   -2/n, -2/n, -3/n, -3/n ];
            OBJ19 = [ 18/n, 20/n, 20/n, 18/n;   -2/n, -2/n,-11/n,-11/n ];
            OBJ20 = [ 20/n, 27/n, 27/n, 20/n;   -9/n, -9/n,-11/n,-11/n ];
            OBJ21 = [ 22/n, 25/n, 25/n, 22/n;   -5/n, -5/n, -7/n, -7/n ];
            OBJ22 = [ 25/n, 27/n, 27/n, 25/n;   -4/n, -4/n, -7/n, -7/n ];
            OBJ23 = [ 29/n, 31/n, 31/n, 29/n;    2/n,  2/n,-14/n,-14/n ];
            
            
            % Create the polygon world
            obj = obj@PolygonWorld(OBJ1, OBJ2, OBJ3, OBJ4, OBJ5, OBJ6, OBJ7, OBJ8, OBJ9, OBJ10, OBJ11, OBJ12, OBJ13, OBJ14, OBJ15, OBJ16, OBJ17, OBJ18, OBJ19, OBJ20, OBJ21, OBJ22, OBJ23);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
        end
        
        
    end
end

