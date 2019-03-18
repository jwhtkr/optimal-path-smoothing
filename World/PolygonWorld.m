classdef PolygonWorld
        
    properties (SetAccess = protected, GetAccess = public)
        polygons = {}; % Stores a cell structure of the polygons
        n_polygons % Stores the number of polygons that exist
    end
    
    properties (SetAccess = protected, GetAccess = protected)
        halfPlanes = {};    % hold the a, b, c values for each polygon as a matrix
    end
    
    methods
        function obj = PolygonWorld()
            % Define verticies for the polygons
            V1 = [6, 5, 1; 1, 6, 4];
            V2 = [17, 21, 21, 17; 1, 1, 7, 7];
            V3 = [7, 8, 13, 14, 13, 8; -1.5, -3, -3, -1.5, 0, 0];
            obj.polygons{1} = V1;
            obj.polygons{2} = V2;
            obj.polygons{3} = V3;
            
            % Define the halfplanes
            for p = 1:length(obj.polygons)
                V = obj.polygons{p};
                H = []; % Holds the halfplanes for the polygon
                
                % Loop through all the vertices in V
                for k = 1:(size(V,2)-1)
                    % Calculate the halfplane
                    [a, b, c] = obj.createHalfPlane(V(:,k), V(:,k+1));
                    %obj.plotHalfplane(a, b, c);
                    
                    % Add the abc values onto the end of the set of halfplanes
                    H = [H; a, b, c]; 
                end
                
                % Get the final connecting verticies                
                [a, b, c] = obj.createHalfPlane(V(:,end), V(:,1));
                %obj.plotHalfplane(a, b, c);

                % Add the abc values onto the end of the set of halfplanes
                H = [H; a, b, c]; 
                
                % Store the set of halfplanes
                obj.halfPlanes{p} = H;                
            end
            
            % Store data for polygon world
            obj.n_polygons = length(obj.polygons);
            
        end
        
        function plotWorld(obj, ax)
            for k = 1:length(obj.polygons)
                % Extract points
                V = obj.polygons{k};
                
                % plot points
                patch(ax, V(1,:), V(2,:), 'r', 'EdgeColor', 'r', 'FaceColor', 'w');
                hold on;
            end
            
        end
        
        function result = insideObstacles(obj, x, y)
            % Evaluate all the obstacle regions
            result = false;
            for p = 1:length(obj.polygons)
                result = result || obj.insideObstacle(x, y, obj.halfPlanes{p});
            end
        end
        
    end
    
    methods
        function result = insideObstacle(obj, x, y, H)
            % Evaluate all of the half planes in H
            result = true;
            for i = 1:size(H,1)
                % Extract halfplane parameters
                a = H(i,1);
                b = H(i,2);
                c = H(i,3);
                
                % Evaluate the halfplane
                result = result && obj.halfPlanePredicate(x,y,a,b,c);                  
             
            end
        end
        
        function result = halfPlanePredicate(obj, x, y, a, b, c)
            % Logical predicate for evaluating a halfplane
            if obj.evalHalfplane(a, b, c, x, y) <= 0
                result = true;
            else
                result = false;
            end
        end
        
        function [a, b, c] = createHalfPlane(obj, v1, v2)
            % Extract values
            x1 = v1(1);
            y1 = v1(2);
            x2 = v2(1);
            y2 = v2(2);
            
            % Calculate a and b
            a = y2 - y1;
            b = x1 - x2;
            
            % Calculate c
            c = -a*x1 - b*y1;
        end
        
        function plotHalfplane(obj, a, b, c)
            [X, Y] = meshgrid([0:.1:22], [-4:.1:8]);
            
            % Evaluate f for all X and Y
            [m,n] = size(X);
            F = zeros(m,n);
            for row = 1:m
                for col = 1:n
                    F(row,col) = obj.evalHalfplane(a,b,c, X(row,col), Y(row,col));
                end
            end
            
            % Plot a contour of the halfplane
            contour(X, Y, F, [-1 0 1], 'ShowText', 'on');
        end
        
        function f = evalHalfplane(obj, a, b, c, x, y)
            f = a*x + b*y + c;
        end
    end
end

