classdef OccupancyGrid < handle
    %OccupancyGrid Stores a 2D occupance grid where each cell is either
    %occupied (1) or free(0)
    
    properties(SetAccess=protected)
        grid % n_rows x n_cols grid of uint8
        n_rows % Number of rows in grid (y values)
        n_cols % Number of columns in grid (x values)
        res % each cell is res x res meters
        tl_corner % 2x1 point indicating the position of the top left cell (grid(1,1))
        x_lim % Limits of the x values
        y_lim % Limits of the y values
    end
    
    methods
        function obj = OccupancyGrid(res, x_lim, y_lim)
            %OccupancyGrid Construct an instance of this class
            %   
            % Inputs:
            %   res: Resolution of each cell
            %   x_lim: 1x2 array where x_lim(1) < x_lim(2)
            %   y_lim: 1x2 array where y_lim(1) < y_lim(2)
           
            % Initialize properties
            obj.res = abs(res);
            obj.tl_corner = [x_lim(1); y_lim(2)];
            
            % Create the grid
            x_diff = x_lim(2)-x_lim(1);
            obj.n_cols = ceil(x_diff/res);
            y_diff = y_lim(2)-y_lim(1);
            obj.n_rows = ceil(y_diff/res);
            obj.grid = uint8(zeros(obj.n_rows, obj.n_cols)); 
            obj.x_lim = x_lim;
            obj.y_lim = y_lim;
        end
        
        function occupied = isOccupied(obj, q)
            %isOccupied returns true if the position is in an occupied
            %cell, false otherwise
            %
            % Inputs:
            %   q: 2x1 position
            
            % Calculate the indices
            [row, col] = obj.positionToIndex(q);
            
            % Determine if it is occupied
            occupied = obj.grid(row, col) > 0;
        end
        
        function setOccupied(obj, q)
            %setOccupied sets the grid location corresponding to the 2x1
            %position q as occupied
            %
            % Inputs:
            %   q: 2x1 position
            
            % Calculate the indices
            [row, col] = obj.positionToIndex(q);
            
            % Determine if it is occupied
            obj.grid(row, col) = 1;
        end
        
        function setFree(obj, q)
            %setFree sets the grid location corresponding to the 2x1
            %position q as free
            %
            % Inputs:
            %   q: 2x1 position
            
            % Calculate the indices
            [row, col] = obj.positionToIndex(q);
            
            % Determine if it is occupied
            obj.grid(row, col) = 0;
        end
        
        function q = indexToPosition(obj, row, col)
            %indexToPosition converts an index into a position
            %
            % Inputs:
            %   row: row index inside grid
            %   col: column index inside grid
            %
            % Outputs:
            %   q: 2x1 position corresponding to the point
            
            q = obj.tl_corner + [obj.res*col; -obj.res*row]; % Negative sign comes from tl being the max y value (positive in row moves negative in y)
        end
        
        function [row, col] = positionToIndex(obj, q)
            %positionToIndex converts a position into an index
            %
            % Inputs:
            %   q: 2x1 position corresponding to the point
            %
            % Outputs:
            %   row: row index inside grid
            %   col: column index inside grid
            
            % Translate the position
            q = q - obj.tl_corner;
            
            % Divide by resolution to get index
            q_ind = q ./ obj.res;
            
            % Round to get integer
            col = round(q_ind(1));
            row = -round(q_ind(2)); % Negative sign comes from tl being the max y value (positive in row moves negative in y)
            
            
            % Saturate to ensure on map
            row = max(1, row);
            row = min(obj.n_rows, row);
            col = max(1, col);
            col = min(obj.n_cols, col);
        end
        
        function testGridAccess(obj)
            % Create a point
            q = obj.tl_corner + [5*rand; -5*rand];
            
            % Calculate indices
            [row, col] = obj.positionToIndex(q);
            
            % Calculate new position based on indices
            q_new = obj.indexToPosition(row, col);
            
            error = norm(q - q_new)
            
        end
    end
end

