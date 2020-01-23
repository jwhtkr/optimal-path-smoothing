classdef CCTrajectoryPlanner < handle
    %BetterUnicycle Implements a unicycle with direct control over the
    %accleration
    
    properties
        dt;
        v;
        max_k;
        max_sigma;
        waypoints;
        waypoints_directon;
        clothoid;
        max_clothoid_deflection;
        zero_deflection_angle;
        
        q_i;
        C_l_inner;
        C_r_inner;
        C_l;
        C_r;
        mu;
        
    end
    
    methods
        function obj = CCTrajectoryPlanner(dt, v, max_k, max_sigma)
            obj.dt = dt;
            obj.v = v;
            obj.max_k = max_k;
            obj.max_sigma = max_sigma;
            
            obj.clothoid = ClothoidGenerator(max_k, v, dt, max_sigma);
            obj.max_clothoid_deflection = 2 * obj.clothoid.traj.psi(end);
            
            % See fig 3 Fraichard
            r_inner = 1/obj.max_k;
            obj.q_i = [obj.clothoid.traj.x(end); obj.clothoid.traj.y(end)];
            angle_perp_to_q_i = pi/2 + obj.clothoid.traj.psi(end);
            R = rotation(angle_perp_to_q_i);
            circle_center = R*[r_inner; 0] + obj.q_i;
            
            obj.C_l_inner = Circle(circle_center(1),circle_center(2),r_inner);
            obj.C_r_inner = Circle(circle_center(1), -circle_center(2),r_inner);
            
            r_out = sqrt(obj.C_l_inner.x^2 + obj.C_l_inner.y^2);
            obj.C_l = Circle(obj.C_l_inner.x,obj.C_l_inner.y,r_out);
            obj.C_r = Circle(obj.C_l_inner.x,-obj.C_l_inner.y,r_out);
            
            obj.mu = atan2(obj.C_l.x,obj.C_l.y);
            
            obj.zero_deflection_angle = -(pi/2 - obj.mu);
        end
        
        function waypoint = update_waypoint_properties(obj, waypoint)
            % Update the circle info for input waypoint. This function uses
            % mostly the class data. Outputs an updated waypoint, might be
            % better as a void CCWaypoint function where the planner class
            % data is passed in.
            
            C_l_center = rotation(waypoint.psi)*([[obj.C_l.x]; [obj.C_l.y]]);
            C_r_center = rotation(waypoint.psi)*([[obj.C_r.x]; [obj.C_r.y]]);
            reverse_offset = 2 * obj.C_l.r * sin(obj.mu);
            C_l_reverse_center = rotation(waypoint.psi)*([obj.C_l.x - reverse_offset; [obj.C_l.y]]);
            C_r_reverse_center = rotation(waypoint.psi)*([obj.C_r.x - reverse_offset; [obj.C_r.y]]);
            
            % update forward circles starting with left
            waypoint.C_l.x = waypoint.x + C_l_center(1);
            waypoint.C_l.y = waypoint.y + C_l_center(2);
            waypoint.C_l.r = obj.C_l.r;
            waypoint.C_l_inner = waypoint.C_l;
            waypoint.C_l_inner.r = obj.C_l_inner.r;
            
            % update circles right
            waypoint.C_r.x = waypoint.x + C_r_center(1);
            waypoint.C_r.y = waypoint.y + C_r_center(2);
            waypoint.C_r.r = obj.C_r.r;
            waypoint.C_r_inner.x = waypoint.x + C_r_center(1);
            waypoint.C_r_inner.y = waypoint.y + C_r_center(2);
            waypoint.C_r_inner.r = obj.C_r_inner.r;
            
            % update reverse circles starting with left
            waypoint.C_l_reverse.x = waypoint.x + C_l_reverse_center(1);
            waypoint.C_l_reverse.y = waypoint.y + C_l_reverse_center(2);
            waypoint.C_l_reverse.r = obj.C_l.r;
            waypoint.C_l_reverse_inner = waypoint.C_l_reverse;
            waypoint.C_l_reverse_inner.r = obj.C_l_inner.r;
            
            % update circles right
            waypoint.C_r_reverse.x = waypoint.x + C_r_reverse_center(1);
            waypoint.C_r_reverse.y = waypoint.y + C_r_reverse_center(2);
            waypoint.C_r_reverse.r = obj.C_r.r;
            waypoint.C_r_reverse_inner = waypoint.C_r_reverse;
            waypoint.C_r_reverse_inner.r = obj.C_r_inner.r;
            
            waypoint.mu = obj.mu;
        end
        
        function cc_turn = build_cc_turn(obj, pose, deflection_angle)
            % builds a CCTurn at pose with ending deflection_angle. A
            % method to verify is to check if cc_turn.psi(end) == pose(3) +
            % deflection_angle. 
            cc_turn = Trajectory2D();
            deflection = abs(deflection_angle);
            direction = sign(deflection_angle);
            
            if direction > 0
                circle = obj.C_l_inner;
            else
                circle = obj.C_r_inner;
            end
            
            if deflection == 0
                % Straight deflection
                % Could probably be an arbitrary distance, but this worked.
                cc_turn = line_trajectory([0, 0], [2 * obj.C_l.r * sin(obj.mu), 0], obj.v, obj.dt);
                cc_turn = cc_turn.rotate_traj(pose.psi,1);
                cc_turn.x = cc_turn.x + pose.x;
                cc_turn.y = cc_turn.y + pose.y;
                
            elseif deflection < obj.max_clothoid_deflection
                % Deflection where k_max is never achieved, so no circular
                % arc is generated.
                
                
                % Truncate clothoid region
                clothoid_end_ind = obj.find_nearest_idx(obj.clothoid.traj.psi, deflection / 2.0);
                temp_traj = obj.clothoid.traj.truncate(clothoid_end_ind);
                
                cc_turn = cc_turn.concatenate(temp_traj);
                cc_turn.s_geo = cc_turn.s(end);
                cc_turn.y = direction * cc_turn.y;
                cc_turn.psi = direction * cc_turn.psi;
                cc_turn.k = direction * cc_turn.k;
                cc_turn.sigma = direction * cc_turn.sigma;

                % specify the final clothoid heading to be correct
                cc_turn.psi(end) = direction * deflection / 2.0;

                % find distance from turn to bisector
                start_angle = direction * (obj.zero_deflection_angle - 2 * obj.mu);
                end_angle = direction * obj.zero_deflection_angle + direction * deflection;
                bisector = (start_angle + end_angle) / 2.0;
                d = (cc_turn.y(end) - circle.y) * cos(bisector) - (cc_turn.x(end) - circle.x) * sin(bisector);
                r = abs(d / sin(bisector));
                cc_turn.x = cc_turn.x + r;

                % create a linear trajectory to complete clothoid
                line_traj = line_trajectory([0, 0], [cc_turn.x(1), cc_turn.y(1)], obj.v, obj.dt);
                if line_traj.x(end) == 0
                    line_traj.x = line_traj.x + r;
                end
                cc_turn = line_traj.concatenate(cc_turn);
                
            else
                % All traj where k(t) reaches k_max
                cc_turn = cc_turn.concatenate(obj.clothoid.traj);
                
                cc_turn.y = direction * cc_turn.y;
                cc_turn.psi = direction * cc_turn.psi;
                cc_turn.k = direction * cc_turn.k;
                cc_turn.sigma = direction * cc_turn.sigma;
                
                circle_start_angle = atan2(cc_turn.y(end) - circle.y, cc_turn.x(end) - circle.x);
                circle_end_angle = circle_start_angle + (deflection_angle - direction * obj.max_clothoid_deflection)/2;
                
                cc_arc = obj.build_arc_trajectory(circle, circle_start_angle, circle_end_angle, direction);
                
                cc_turn = cc_turn.concatenate(cc_arc);
            end
            
            cc_turn = obj.reflect_traj(cc_turn);
            cc_turn = cc_turn.rotate_traj(pose.psi,1);
            cc_turn.x = cc_turn.x + pose.x;
            cc_turn.y = cc_turn.y + pose.y;
            
            
            cc_turn.w = cc_turn.v .* cc_turn.k;
            % cc_turn.alpha = cc_turn.v .* cc_turn.v .* cc_turn.sigma;
            cc_turn.alpha = cc_turn.sigma .* cc_turn.v;
            cc_turn.update_derivatives();
            cc_turn.t = cc_turn.s ./ cc_turn.v;
            
           
        end
        
        function traj = build_arc_trajectory(obj, circle, start_psi, end_psi, direction)
            % creates a trajectory of an arc, when k == k_max
            % Inputs:
            %   circle: the circle that the traj is following
            %   start_psi: the angle at the beginning of traj
            %   end_psi: the angle at the end of traj
            %   direction: whether this is a left, 1, or right, -1, turn
            % Output: arc trajectory
            
            traj = Trajectory2D();
            traj.ds = obj.dt*obj.v;
            start_psi = mod(start_psi, direction * pi*2);
            end_psi = mod(end_psi, direction * pi*2);
            
            if direction * end_psi < direction* start_psi
                end_psi = end_psi + direction *2*pi;
            end
            
            traj.s_geo = circle.r * abs(start_psi - end_psi);
            num = round(traj.s_geo / traj.ds,0) - 1;
            psi = start_psi:(end_psi - start_psi)/num:end_psi;
            n = length(psi);
            traj.x = circle.r * cos(psi) + circle.x;
            traj.y = circle.r * sin(psi) + circle.y;
            
            traj.psi = mod(psi+direction*pi/2+pi, 2*pi) - pi;
            traj.k = ones(1,n) * direction / circle.r;
            traj.s = circle.r * abs(psi - start_psi);
            traj.v = ones(1,n)*obj.v;                           %% default v value
            traj.sigma = zeros(1,n);
            traj.a = zeros(1,n);
            traj.update_derivatives();
            traj.t = traj.s./traj.v;
        end
        
        function new_traj = reflect_traj(obj, traj)
            % reflects a traj about it's endpoint, used primarily for when
            % deflection_angle > 0
            % Input:
            %   traj: the trajectory to be reflected. normally not an
            %   entire path, just a single turn
            % Output:
            %   new_traj: a trajectory containing traj and it's reflection
            
            reflection_psi = traj.psi(end) + pi/2;
            reflection_pt = [cos(reflection_psi); sin(reflection_psi)];
            
            clothoid_end_pt = [traj.x(end);traj.y(end)];
            reflection_pt_perp = [-reflection_pt(2); reflection_pt(1)];
            R = rotation(reflection_psi);
            
            shifted_clothoid = [flip(traj.x) - clothoid_end_pt(1); flip(traj.y) - clothoid_end_pt(2)];
            
            error = [(shifted_clothoid'*reflection_pt)'; (-shifted_clothoid'* reflection_pt_perp)'];
            
            reflected_clothoid = R*error +clothoid_end_pt;
            
            cc_reflect = traj.reverse_traj();
            cc_reflect.x = reflected_clothoid(1,:);
            cc_reflect.y = reflected_clothoid(2,:);
            cc_reflect.psi = cc_reflect.psi + 2 * traj.psi(end);
            
            new_traj = traj.concatenate(cc_reflect);
        end
        
        function new_traj = squeeze_turn(obj,waypoint_0, waypoint_1,traj)
            % This function squeezes a turn into the corner of the path
            % diagram. Which is constructed but plotting the path variable
            % in CCPathGenerator.
            
            % rename variables for ease of reading
            psi0 = waypoint_0.psi;
            psi1 = waypoint_1.psi;
            v1 = [waypoint_1.x; waypoint_1.y];
            R = rotation(-psi0);
            
            % rotate turn traj to parallel with x axis
            traj = traj.rotate_traj(-psi0,2); % plot(traj.x,traj.y);
            v1 = R*v1;
            % find the slope at waypoint_1
            m = tan(psi1-psi0);
            pose_end = [traj.x(end); traj.y(end); traj.psi(end)];
            % eq obtained from basic line eq y = m(x + x0) + y0
            x_at_intersection = (pose_end(2)- v1(2))/m + v1(1);
            
            dx = x_at_intersection - traj.x(end);
            traj.x = traj.x + dx; % plot(traj.x,traj.y);
            
            % return to proper orientation
            traj = traj.rotate_traj(psi0,2);
            
            
            new_traj = traj;            
        end
        
        function ind = find_nearest_idx(obj, array, value)
            % Finds one indice before the closest to value
            [m, ind] = min(abs(array - value));
            ind = ind - 1;
        end
    end
end