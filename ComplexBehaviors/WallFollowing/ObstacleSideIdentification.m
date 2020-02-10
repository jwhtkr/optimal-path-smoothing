classdef ObstacleSideIdentification < handle
    %ObstacleSideIdentification This class is used to repeatedly identify
    %obstacles that are on the side of a vehicle
    
    properties (SetAccess = protected)
        initialized = false; % flag turned to true once initialization has been successful
        q_closest = []; % Stores the closest point found to the vehicle on the given side        
        dist_cont % Distance for a continous obstacle
        dist_cont_x2 % Two times the distance of a continuous obstacle
    end
    
    methods
        function obj = ObstacleSideIdentification(dist_cont)
            %ObstacleSideIdentification Construct an instance of this class
            %
            % Inputs:
            %   dist_cont: Distance for a continous obstacle
            
            % Store input variables
            obj.dist_cont = dist_cont;
            obj.dist_cont_x2 = dist_cont*2;
        end
        
        function success = initializePointOnWall(obj, q_wall, q_veh, orient)
            %initializePointOnWall A point on the wall that is closest
            %
            % Inputs:
            %   q_wall: 2xm matrix of points where the first row contains the x values
            %         and the second row contains the y values
            %   q_veh: 2x1 position vector of the vehicle
            %   orient: orientation of the vehicle
            %
            % Outputs:
            %   success: true if a valid point was found, false otherwise
            
            % Calculate line
            [q0, vl] = leastSquaresLine(q_wall); % Point on line (q0) and vector of line (vl)
            
            % Adjust v to point in same direction as vehicle
            orien_vec = [cos(orient) sin(orient)]; % Orientation vector
            if orien_vec*vl < 0 % The line is pointing in the wrong direction
                vl = -vl;
            end
            
            % Get the point on the line that is closest to the vehicle
            q_closest_line = q0 + vl.*(vl'*(q_veh-q0));
            
            % Find the point in q_wall that is closest to the line point
            n_pnts = size(q_wall, 2);
            d_closest = inf;
            obj.q_closest = q_closest_line;
            for k = 1:n_pnts
                % Calculate distance to point k
                d = norm(q_closest_line-q_wall(:,k));
                
                % Store the point if it is the closest found
                if d < d_closest
                    d_closest = d;
                    obj.q_closest = q_wall(:,k);
                end
            end
            
            % Determine if the point is valide
            if d_closest > obj.dist_cont_x2
                success = false;
            else
                success = true;
            end
            obj.initialized = success;                
        end
        
        function [q_wall, ind_sens] = findContinguousWall(obj, q_sens, q_veh)
            %findContinguousWall finds the points within q_sens that form a
            % contiguous wall with the closest point already stored (i.e. q_closest).
            %
            % It is assumed that q_sens forms an ordered sensor list around
            % the vehicle so the sensor list can be iterated once through
            %
            % The closest point on the wall compared to the vehicle
            % position will be stored for future updates (including the
            % previous closest point)
            %
            % Inputs:
            %   q_sens: 2xm matrix of points where the first row contains the x values
            %           and the second row contains the y values
            %   q_veh: 2x1 position of the vehicle
            %
            % Outputs:
            %   q_wall: 2xn matrix of points that belong to the wall
            %           defined by
            %   ind_sens: Indices of q_wall within q_sens
            
            % Check to see if the closest point has been initialized
            if ~obj.initialized
                error('Cannot find contingous wall without the point being initialized');
            end
            
            % Find the point in q_sens that is closest to the previous
            % sensed wall point
            d_closest = inf;
            ind_closest = inf;
            q_close_sense = [inf; inf];
            n_pnts = size(q_sens, 2);
            for k = 1:n_pnts
                % Calculate the distance to point k
                d = norm(q_sens(:,k)- obj.q_closest);
                
                % Store the closest point
                if d < d_closest
                    d_closest = d;
                    ind_closest = k;
                    q_close_sense = q_sens(:,k);
                end
            end
            
            if d_closest > obj.dist_cont_x2
                warning('No point was found sufficiently close to last detected wall');
                q_wall = [];
                ind_sens = [];
                obj.initialized = false;
            end
            
            % Find the continuous portion going forward
            ind_added = zeros(1, n_pnts);
            ind_added(ind_closest) = true;
            q_wall_forward = zeros(2, n_pnts); % Initialize point storage to as big as possible
            q_wall_forward(:,1) = q_close_sense;
            ind_wall_forward = zeros(1, n_pnts); % Index storage for wall points
            ind_wall_forward(1) = ind_closest;
            n_forward = 1;
            for k = (ind_closest+1):n_pnts % Search to the end of the sensor list
                % Don't look at the point if it has already been added
                if ind_added(k)
                    continue;
                end
                
                % See if the point is within twice the continous distance
                d = norm(q_sens(:,k) - q_wall_forward(:,n_forward));
                if d < obj.dist_cont_x2
                    n_forward = n_forward+1; % Increment the index of the forward point
                    q_wall_forward(:,n_forward) = q_sens(:,k); % Store point
                    ind_wall_forward(n_forward) = k; % Store index
                    ind_added(k) = true;
                end
            end
            for k = 1:(ind_closest-1) % Search from the start to the 
                if ind_added(k)
                    continue;
                end
                
                % See if the point is within twice the continous distance
                d = norm(q_sens(:,k) - q_wall_forward(:,n_forward));
                if d < obj.dist_cont_x2
                    n_forward = n_forward+1; % Increment the index of the forward point
                    q_wall_forward(:,n_forward) = q_sens(:,k);
                    ind_wall_forward(n_forward) = k; % Store index
                    ind_added(k) = true;
                end
            end            
            
            % Find the continous section going backward
            q_wall_backward = zeros(2, n_pnts); % Initialize point storage to as big as possible
            q_wall_backward(:,1) = q_close_sense;
            ind_wall_back = zeros(1, n_pnts); % Index storage for wall points
            ind_wall_back(1) = ind_closest;
            n_back = 1;
            for k = (ind_closest-1):-1:1 % Search to the end of the sensor list going backwards
                if ind_added(k)
                    continue;
                end
                
                % See if the point is within twice the continous distance
                d = norm(q_sens(:,k) - q_wall_backward(:,n_back));
                if d < obj.dist_cont_x2
                    n_back = n_back+1; % Increment the index of the forward point
                    q_wall_backward(:,n_back) = q_sens(:,k); % Store point
                    ind_wall_back(n_back) = k; % Store index
                    ind_added(k) = true;
                end
            end
            for k = n_pnts:-1:(ind_closest+1) % Search from the final to the closest
                if ind_added(k)
                    continue;
                end
                
                % See if the point is within twice the continous distance
                d = norm(q_sens(:,k) - q_wall_backward(:,n_back));
                if d < obj.dist_cont_x2
                    n_back = n_back+1; % Increment the index of the forward point
                    q_wall_backward(:,n_back) = q_sens(:,k);
                    ind_wall_back(n_back) = k; % Store index
                    ind_added(k) = true;
                end
            end
            
            
            % Merge the lists together
            q_wall = [q_wall_backward(:, n_back:-1:2), q_wall_forward(:,1:n_forward)];
            ind_sens = [ind_wall_back(n_back:-1:2), ind_wall_forward(1:n_forward)];
            if size(q_wall, 2) > n_pnts
                warning('The vehicle is inside a point with continuous wrap around');
                q_wall = q_sens;
                ind_sens = 1:n_pnts;
            end
            
            % Store the point closest to the vehicle
            d_closest = norm(q_veh - obj.q_closest);
            for k = 1:size(q_wall, 2)
                % Calculate distance to point
                d = norm(q_veh-q_wall(:,k));
                
                % Store point
                if d <= d_closest
                    d_closest = d;
                    obj.q_closest = q_wall(:,k);
                end
            end
        end
        
        function setClosestPoint(obj, q_closest)
            %setClosestPoint sets the closest point to be used to find a
            %continguous wall
            %
            % Inputs:
            %   q_closest: 2x1 position of a point on the wall of interest
            
            obj.q_closest = q_closest;
            obj.initialized = true;
        end
    end
end

