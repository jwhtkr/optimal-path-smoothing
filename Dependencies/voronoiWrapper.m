classdef voronoiWrapper
    % This Class uses MatLAb's standard voronoi function to make a voronoi virtual structure.
    properties
        Q; % location of the points that define the voronoi diagram [x1 x2 x3; y1 y2 y3]
        V; % Contains the line definitions [x11 x12 x21 x22 x31 x32 ... xn1 xn2; y11 y12 y21 y22 y31 y32 ... yn1 yn2]
        % Each line is defined by 2 points, [xi1 xi2; yi1 yi2]
        map; % provides the relationship between Q(:,k) and the lines associated with point's cell.
    end
    methods
        
        function obj = voronoiWrapper(Q)
            [x,y] = obj.Q_to_xy(Q);
            [vx,vy] = voronoi(x',y');
            obj.V = obj.reformatVoronoi(vx,vy);
            obj.Q = Q;
            obj.map = obj.mapping();
        end
        
        function [x,y] = Q_to_xy(obj, Q)
            x = Q(1,:);
            y = Q(2,:);
        end
        
        function map = mapping(obj)
            map = cell(length(obj.Q),1);
            % Checks distance to each line's endpoints
            for k = 1:length(obj.Q)
                for j = 1:length(obj.V)/2
                    d1(k,2*j-1) = obj.dist_to_vertex(obj.V(:,2*j-1),obj.Q(:,k));
                    d1(k,2*j) = obj.dist_to_vertex(obj.V(:,2*j),obj.Q(:,k));
                end
            end
            
            for k = 1:length(obj.V)/2
                I1 = find(round(d1(:,2*k-1),10) == round(min(d1(:,2*k-1)),10));
                I2 = find(round(d1(:,2*k),10) == round(min(d1(:,2*k)),10));
                I = intersect(I1,I2);
                for i = 1:length(I)
                    map{I(i)} = [map{I(i)} k];
                end
            end
        end
        
        function V = reformatVoronoi(obj,vx,vy)
            for k = 1:length(vx)
                V(:,2*k-1) = [vx(1,k); vy(1,k)];
                V(:,2*k) = [vx(2,k); vy(2,k)];
            end
        end
        
        function Q = points(obj,x,y)
            for k = 1:length(x)
                Q(:,k) = [x(k); y(k)];
            end
        end
        
        % basic distance between 2 points
        function d = dist_to_vertex(obj,v,q)
            d = sqrt((v(1) - q(1))^2 + (v(2)-q(2))^2);
        end
        
        % Returns the distance of an agent from it's barrier, should be a
        % 1xn where n is the number of lines in agent_num's cell
        function D = dist_to_barrier(obj, q, agent_num, leader_heading, leader_pose)
            D = [];
            R = obj.rotation(leader_heading);
            V = R*obj.V + leader_pose*ones(1,length(obj.V));
            for k = 1:length(obj.map{agent_num})
                D = [D obj.dist_to_line(V(:,2*obj.map{agent_num}(:,k)-1),V(:,2*obj.map{agent_num}(:,k)),q)];
            end
        end
        
        % Rotation Matrix
        function R = rotation(obj,th)
            R = [cos(th), -sin(th); sin(th) cos(th)];
        end
        
        % Distance to a line from a point
        function d = dist_to_line(obj, v1, v2, pt)
            a = [v1;0] - [v2;0];
            b = [pt;0] - [v2;0];
            d = norm(cross(a,b)) / norm(a);
        end
        
        % distance and normal vector of a point from a line
        function [d,n] = normal(obj, p1, p2, q)
            p = p2 - p1;
            q_p = q - p1;
            p = p/norm(p);
            q_hat = dot(q_p,p)*p;
            q_bar = q_p-q_hat;
            d = norm(q_bar);
            n = q_bar/d;
        end
        
        
        % Returns distance and normal vectors of a point from it's cell boundaries
        function [D,N] = dist_and_norm(obj, q, agent_num, leader_heading, leader_pose)
            D = [];
            N = [];
            R = obj.rotation(leader_heading);
            V = R*obj.V + leader_pose*ones(1,length(obj.V));
            for k = 1:length(obj.map{agent_num})
                [d,n] = obj.normal(V(:,2*obj.map{agent_num}(:,k)-1),V(:,2*obj.map{agent_num}(:,k)),q);
                D = [D d];
                N = [N n];
            end
        end
    end
end
