classdef CCPathGenerator < handle
    % CCPathGenerator generates a Continuous Curvature Path (CCPath/CCTraj)
    
    properties
        planner;
        deflection_angle;
        traj;
    end
    
    methods
        function obj = CCPathGenerator(path,v,dt,k,sig)
            % Might be good for future iterations to have v and dt be
            % inputs as well.
            % 
            % Inputs:
            %   path: an array of path points in desired order
            %       i.g. path = [q0,q1,...,qn] where q = [x y]'
            % Outputs: CCPathGenerator object
            
            
            [n m ] = size(path);
%             plot(path(:,1),path(:,2)); hold on;
            

            obj.planner = CCTrajectoryPlanner(dt,v,k,sig);
            obj.traj = Trajectory2D();
            [waypoints, deflection_angles] = obj.path_to_waypoints(path);
            
            
            for k = 1:n-2
                waypoint_0 = CCWaypoint(waypoints(k,:));
                waypoint_1 = CCWaypoint(waypoints(k+1,:));
                waypoint_2 = CCWaypoint(waypoints(k+2,:));
                obj.planner.update_waypoint_properties(waypoint_0);
                
                deflection_angle = deflection_angles(k);
                if deflection_angle == 0
                    turn_traj = Trajectory2D();
                    turn_traj.x = waypoint_0.x;
                    turn_traj.y = waypoint_0.y;
                else
                    turn_traj = obj.planner.build_cc_turn(waypoint_0, deflection_angle);
                    turn_traj = obj.planner.squeeze_turn(waypoint_0, waypoint_1, waypoint_2, turn_traj);
%                     turn_traj = turn_traj.rotate_traj(waypoint_0.psi,1);
%                     turn_traj.x = turn_traj.x + waypoint_0.x;
%                     turn_traj.y = turn_traj.y + waypoint_0.y;
                end
                if k == 1
                    line_traj = line_trajectory([waypoint_0.x;waypoint_0.y], [turn_traj.x(1); turn_traj.y(1)], v, dt);
                elseif k == n-1
                    line_traj = line_trajectory([obj.traj.x(end);obj.traj.y(end)],[waypoint_1.x;waypoint_1.y], v, dt);
                else
                    line_traj = line_trajectory([obj.traj.x(end);obj.traj.y(end)], [turn_traj.x(1); turn_traj.y(1)], v, dt);
                end
                
                if k == n-1
                    obj.traj = obj.traj.concatenate(line_traj);
                else
                    obj.traj = obj.traj.concatenate(line_traj);
                    obj.traj = obj.traj.concatenate(turn_traj);
                end
            end
            obj.traj.update_derivatives();
            obj.traj.dt = dt;
%             plot(obj.traj.x, obj.traj.y);
        end
        
        
        
        function [waypoints, deflection_angles] = path_to_waypoints(obj, path)
            % Function takes a path and creates mock waypoints at each
            % point. This is more useful to get deflection angles and
            % orientation of the future path.
            
            [n,m] = size(path);
            waypoints = zeros(n,m+1);
            deflection_angles = zeros(n,1);
            for k = 1:n
                x = path(k,1);
                y = path(k,2);
                waypoints(k,1:2) = path(k,:);
                if k == n
                    waypoints(k,3) = atan2(y - path(k-1,2), x - path(k-1,1));
                else
                    waypoints(k,3) = atan2(path(k+1,2) - y, path(k+1,1) - x);
                end
                
                if k < n-1
                    deflection_angles(k) = atan2(path(k+2,2) - path(k+1,2),path(k+2,1) - path(k+1,1)) - atan2(path(k+1,2) - y, path(k+1,1) - x);
%                     deflection_angles(k) = atan((path(k+2,2) - path(k+1,2))/(path(k+2,1) - path(k+1,1))) - atan((path(k+1,2) - y)/ (path(k+1,1) - x))
                    if deflection_angles(k) < -pi
                        deflection_angles(k) = deflection_angles(k) + 2*pi;
                    elseif deflection_angles(k) > pi
                        deflection_angles(k) = deflection_angles(k) - 2*pi;
                    end
                else
                    deflection_angles(k) = 0;
                end
                
            end
        end
        
    end
end