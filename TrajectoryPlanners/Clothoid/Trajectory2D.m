classdef Trajectory2D < handle
    %BetterUnicycle Implements a unicycle with direct control over the
    %accleration
    
    properties
        % Position variables
        x;
        xdot;
        xddot;
        xdddot;
        xddddot;
        
        y;
        ydot;
        yddot;
        ydddot;
        yddddot;
        
        % Clothoid parameters
        ds; % v*dt, used in calculation of length of clothoid
        s_geo; % ???
        s; % path length
        t; % time
        dt; % time step
        transitions; % indicies in which transition from clothoid to line to arc        
        cloth_len; % total index length of clothoid
        
        % Translational variables
        v; % Velocity
        a; % Acceleration
        j; % Jerk
        
        % Rotational variables
        psi; % Orientation
        w; % Angular velocity
        alpha; % Angular acceleration
        zeta; % Angular jerk
        
        % Curvature variables
        k; % Curvature
        sigma; % curvature rate
        gamma; % Curvature acceleration
    end
        
    methods
        function obj = Trajectory2D()
            obj.ds = [];
            obj.s = [];
            obj.v = [];
            obj.a = [];
            obj.sigma = [];
            obj.gamma = [];
            obj.transitions = [1];
            obj.t = [];
            obj.psi = [];
            obj.k = [];
            
            obj.x = [];
            obj.xdot = [];
            obj.xddot = [];
            obj.xdddot = [];
            obj.xddddot = [];
            
            obj.y = [];
            obj.ydot = [];
            obj.yddot = [];
            obj.ydddot = [];
            obj.yddddot = [];
            obj.s_geo = 0;
        end
        
        function copy(obj, traj)
        %copy copies the trajectory in traj to this object
            % Position variables
            obj.x = traj.x;
            obj.xdot = traj.xdot;
            obj.xddot = traj.xddot;
            obj.xdddot = traj.xdddot;
            obj.xddddot = traj.xddddot;

            obj.y = traj.y;
            obj.ydot = traj.ydot;
            obj.yddot = traj.yddot;
            obj.ydddot = traj.ydddot;
            obj.yddddot = traj.yddddot;

            % Clothoid parameters
            obj.ds = traj.ds; % v*dt, used in calculation of length of clothoid
            obj.s_geo = traj.s_geo; % ???
            obj.s = traj.s; % path length
            obj.t = traj.t; % time
            obj.dt = traj.dt; % time step
            obj.transitions = traj.transitions; % indicies in which transition from clothoid to line to arc        
            obj.cloth_len = traj.cloth_len; % total index length of clothoid

            % Translational variables
            obj.v = traj.v; % Velocity
            obj.a = traj.a; % Acceleration
            obj.j = traj.j; % Jerk

            % Rotational variables
            obj.psi = traj.psi; % Orientation
            obj.w = traj.w; % Angular velocity
            obj.alpha = traj.alpha; % Angular acceleration
            obj.zeta = traj.zeta; % Angular jerk

            % Curvature variables
            obj.k = traj.k; % Curvature
            obj.sigma = traj.sigma; % curvature rate
            obj.gamma = traj.gamma; % Curvature acceleration
        end
        
        function update(obj,x)
            obj.x = x(1,:);
            obj.y = x(2,:);
            obj.psi = x(3,:);
            obj.k = x(4,:);
            
            obj.v = ones(1,obj.cloth_len)*obj.v;
            obj.sigma = ones(1,obj.cloth_len)*obj.sigma;
            obj.gamma = zeros(1,obj.cloth_len)*obj.gamma;
            obj.a = zeros(1,obj.cloth_len);
            obj.j = zeros(1,obj.cloth_len);
            obj.s_geo = obj.s;
            obj.t = obj.s ./ obj.v;
            
            obj.update_derivatives();
        end
        
        function updateTrajWithPositionalValues(obj)
        %updateTrajWithPositionalValues will use the positional values
        %(i.e., x ... xddddot, y ... yddddot to update the following
        %variables of the trajectory:
        %   v, a, j, psi, w, alpha, zeta, k, sigma, gamma   
        
            % Update translational variables
            obj.v = sqrt(obj.xdot.^2 + obj.ydot.^2);
            obj.a = (obj.xdot.*obj.xddot + obj.ydot.*obj.yddot) ./ obj.v;
            obj.j = (obj.xddot.^2 + obj.xdddot.*obj.xdot + obj.yddot.^2 + obj.ydddot.*obj.ydot - obj.a.^2)./obj.v;
            
            % Update Rotational variables
            obj.psi = atan2(obj.ydot,obj.xdot);
            obj.w = (obj.xdot.*obj.yddot - obj.ydot.*obj.xddot)./(obj.v.^2);
            %obj.alpha = (obj.xdot.*obj.ydddot - obj.ydot.*obj.xdddot)./(obj.v.^2) - 2*obj.a.*obj.w./obj.v;
            obj.alpha = (obj.xdot.*obj.ydddot - obj.ydot.*obj.xdddot)./(obj.v.^2) - 2*obj.a.*(obj.xdot.*obj.yddot - obj.ydot.*obj.xddot)./(obj.v.^3);
            obj.zeta = (obj.yddddot.*obj.xdot + obj.ydddot.*obj.xddot - obj.xdddot.*obj.yddot - obj.xddddot.*obj.ydot)./obj.v.^2 ...
                        -2*obj.a.*(obj.ydddot.*obj.xdot - obj.xdddot.*obj.ydot)./obj.v.^3 ...
                        +(2*obj.a.^2.*obj.w -2*obj.w.*obj.j - 2*obj.a.*obj.alpha)./obj.v;
                    
            % Update curvature variables
            obj.k = obj.w ./ obj.v;
            obj.sigma = (obj.v.*obj.alpha - obj.w.*obj.a)./(obj.v.^2);
            obj.gamma = (obj.v.*obj.zeta - obj.w.*obj.j)./(obj.v.^2) - ...
                2.*(obj.v.*obj.alpha - obj.w.*obj.a).*obj.a./(obj.v.^3);
            
            % Update the length variables (why?, seems like this should be removed)
            obj.cloth_len = length(obj.x);
            obj.t = 0:obj.dt:(obj.cloth_len-1)*obj.dt;
        end
        
        function update_derivatives(obj)   
            obj.xdot = obj.v .* cos(obj.psi);
            obj.xddot = -obj.v.^2 .* obj.k .* sin(obj.psi) - 2 * obj.v .* obj.a .* obj.k .* sin(obj.psi);
            obj.xdddot = - obj.v.^2 .* obj.sigma .* sin(obj.psi) - obj.v.^3 .* obj.k .^ 2 .* cos(obj.psi);
            obj.xddddot = obj.v.^3 .* obj.k .* (obj.v.*obj.k.^2.*sin(obj.psi) - 3*obj.sigma.*cos(obj.psi)) - obj.v.^2.*obj.gamma.*sin(obj.psi);
            
            obj.ydot = obj.v .* sin(obj.psi);
            obj.yddot = obj.v .^ 2 .* obj.k .* cos(obj.psi) - 2 * obj.v .* obj.a .* obj.k .* cos(obj.psi);
            obj.ydddot = obj.v .^ 2 .* obj.sigma .* cos(obj.psi) - obj.v .^ 3 .* obj.k .^ 2 .* sin(obj.psi);
            obj.yddddot = -obj.v.^3 .* obj.k .* (obj.v.*obj.k.^2.*cos(obj.psi) + 3*obj.sigma.*sin(obj.psi)) + obj.v.^2.*obj.gamma.*cos(obj.psi);
            
            obj.j = (obj.xddot.^2 + obj.xdddot.*obj.xdot + obj.yddot.^2 + obj.ydddot.*obj.ydot - obj.a.^2)./obj.v;
            
            obj.w = (obj.xdot.*obj.yddot - obj.ydot.*obj.xddot)./obj.v;
            obj.alpha = (obj.xdot.*obj.ydddot - obj.ydot.*obj.xdddot)./obj.v - 2*obj.a.*obj.w./obj.v;
            obj.zeta = (obj.yddddot.*obj.xdot + obj.ydddot.*obj.xddot - obj.xdddot.*obj.yddot - obj.xddddot.*obj.ydot)./obj.v.^2 ...
                        -2*obj.a.*(obj.ydddot.*obj.xdot - obj.xdddot.*obj.ydot)./obj.v.^3 ...
                        +(2*obj.a.^2.*obj.w -2*obj.w.*obj.j - 2*obj.a.*obj.alpha)./obj.v;
        end
        
        function temp_traj = concatenate(obj, new_traj)
            
            temp_traj = Trajectory2D();
            
            
            if isempty(obj.x)
                range = 0;
            else
                range = length(obj.x)-1;
            end
            
            if isempty(new_traj.s) 
                temp_traj = obj;
            end
            
            new_traj_range = length(new_traj.x);
            if isempty(obj.s)
                temp_traj.s = [obj.s new_traj.s(1:end)];
            else
                temp_traj.s = [obj.s(1:range) new_traj.s(1:end)];
            end
            
            temp_traj.transitions = [obj.transitions new_traj.transitions + range];
            
            temp_traj.x = [obj.x(1:range) new_traj.x];
            temp_traj.xdot = [obj.xdot(1:range) new_traj.xdot];
            temp_traj.xddot = [obj.xddot(1:range) new_traj.xddot];
            temp_traj.xdddot = [obj.xdddot(1:range) new_traj.xdddot];
            temp_traj.xddddot = [obj.xddddot(1:range) new_traj.xddddot];
            
            
            temp_traj.y = [obj.y(1:range) new_traj.y];
            temp_traj.ydot = [obj.ydot(1:range) new_traj.ydot];
            temp_traj.yddot = [obj.yddot(1:range) new_traj.yddot];
            temp_traj.ydddot = [obj.ydddot(1:range) new_traj.ydddot];
            temp_traj.yddddot = [obj.yddddot(1:range) new_traj.yddddot];
            
            temp_traj.psi = [obj.psi(1:range) new_traj.psi];
            
            temp_traj.k = [obj.k(1:range) new_traj.k];
            temp_traj.sigma = [obj.sigma(1:range) new_traj.sigma];
            temp_traj.gamma = [obj.gamma(1:range) new_traj.gamma];
            temp_traj.v = [obj.v(1:range) new_traj.v];
            temp_traj.a = [obj.a(1:range) new_traj.a];
            temp_traj.s_geo = obj.s_geo + new_traj.s_geo;
            temp_traj.t = temp_traj.s ./ temp_traj.v;
            
            temp_traj.alpha = [obj.alpha(1:range) new_traj.alpha]; % Angular acceleration
            temp_traj.zeta = [obj.zeta(1:range) new_traj.zeta]; % Angular jerk
            
            temp_traj.j = [obj.j(1:range) new_traj.j]; % Jerk
            temp_traj.w = [obj.w(1:range) new_traj.w]; % Angular velocity
            
            temp_traj.cloth_len = length(temp_traj.x);
            temp_traj.dt = new_traj.dt;
            temp_traj.ds = new_traj.ds;
        
        end
        
        function traj = reverse_traj(obj)
            traj = Trajectory2D();
            traj = traj.concatenate(obj);
            
            traj.x = flip(obj.x);
            traj.xdot = flip(obj.xdot) - (max(traj.xdot) - min(traj.xdot));
            min_val = min(traj.xdot);
            max_val = max(traj.xdot);
            traj.xdot = -(traj.xdot - max_val) + min_val;
            traj.xddot = flip(obj.xddot);
            traj.xdddot = -flip(obj.xdddot);
            traj.xddddot = flip(obj.xddddot);
            
            traj.y = flip(obj.y);
            traj.ydot = flip(obj.ydot) - (max(traj.ydot) - min(traj.ydot));
            min_val = min(traj.ydot);
            max_val = max(traj.ydot);
            traj.ydot = -(traj.ydot - max_val) + min_val;
            traj.yddot = flip(obj.yddot);
            traj.ydddot = -flip(obj.ydddot);
            traj.yddddot = flip(obj.yddddot);
            
            traj.psi = -flip(obj.psi);
            traj.w = flip(obj.w);
            traj.alpha = -flip(obj.alpha);
            
            traj.v = flip(obj.v);
            traj.a = -flip(obj.a);
            traj.j = flip(obj.j);
            traj.zeta = -flip(obj.zeta);
            
            traj.k = flip(obj.k);
            traj.sigma = -flip(obj.sigma);
            
            traj.s = abs(flip(obj.s) - max(obj.s));
            m = length(obj.transitions)-1;
%             if length(obj.transitions)>1
                traj.transitions = length(traj.x) - flip(obj.transitions(end-m:end)) + 1;
%             else
%                 traj.transitions = length(traj.x) - flip(obj.transitions) + 1;
%             end
        end
        
        function tmp_traj = rotate_traj(obj,psi,logic)
            % This Function rotates a traj about it's starting point not
            % about the orgin or global frame.
            
            tmp_traj = Trajectory2D();
            R = rotation(psi);
            pose_dot = [obj.xdot; obj.ydot];
            pose_ddot = [obj.xddot; obj.yddot];
            pose_dddot = [obj.xdddot; obj.ydddot];
            pose_ddddot = [obj.xddddot; obj.yddddot];
     
            switch logic
                case 1
                    pose = [obj.x - obj.x(1); obj.y - obj.y(1)];
                    pose = R*pose;
                    tmp_traj.x = pose(1,:) + obj.x(1);
                    tmp_traj.y = pose(2,:) + obj.y(1);
                case 2
                    pose = [obj.x; obj.y];
                    pose = R*pose;
                    tmp_traj.x = pose(1,:);
                    tmp_traj.y = pose(2,:);
            end
            
            pose_dot = R*pose_dot;
            pose_ddot = R*pose_ddot;
            pose_dddot = R*pose_dddot;
            pose_ddddot = R*pose_ddddot;
            
            tmp_traj.xdot = pose_dot(1,:);
            tmp_traj.ydot = pose_dot(2,:);
            tmp_traj.xddot = pose_ddot(1,:);
            tmp_traj.yddot = pose_ddot(2,:);
            tmp_traj.xdddot = pose_dddot(1,:);
            tmp_traj.ydddot = pose_dddot(2,:);            
            tmp_traj.xddddot = pose_ddddot(1,:);
            tmp_traj.yddddot = pose_ddddot(2,:);
            
            tmp_traj.psi = obj.psi + psi;
            tmp_traj.w = obj.w;
            tmp_traj.alpha = obj.alpha;
            tmp_traj.zeta = obj.zeta; % Angular jerk
            
            tmp_traj.k = obj.k;
            tmp_traj.sigma = obj.sigma;
            tmp_traj.gamma = obj.gamma;
            
            tmp_traj.s = obj.s;
            
            tmp_traj.v = obj.v;
            tmp_traj.a = obj.a;
            tmp_traj.j = obj.j;
            
            tmp_traj.transitions = obj.transitions;
            tmp_traj.ds = obj.ds;
            tmp_traj.dt = obj.dt;
        end
        
        function truncate(obj,ind)

            obj.s = obj.s(1:ind);
            obj.s_geo = 0;
            obj.transitions = [1];
            obj.t = obj.t(1:ind);
            obj.cloth_len = ind;
            
            % Translational variables
            obj.v = obj.v(1:ind);
            obj.a = obj.a(1:ind);
            obj.j = obj.j(1:ind);
            
            % Rotational variables
            obj.psi = obj.psi(1:ind);
            obj.w = obj.w(1:ind);
            obj.alpha = obj.alpha(1:ind);
            obj.zeta = obj.zeta(1:ind);
            
            % Curvature variables
            obj.k = obj.k(1:ind);
            obj.sigma = obj.sigma(1:ind);            
            
            % Position variables
            obj.x = obj.x(1:ind);
            obj.xdot = obj.xdot(1:ind);
            obj.xddot = obj.xddot(1:ind);
            obj.xdddot = obj.xdddot(1:ind);
            obj.xddddot = obj.xddddot(1:ind);
            
            obj.y = obj.y(1:ind);
            obj.ydot = obj.ydot(1:ind);
            obj.yddot = obj.yddot(1:ind);
            obj.ydddot = obj.ydddot(1:ind);
            obj.yddddot = obj.yddddot(1:ind);            
        end
            
        function [q, qdot, qddot, qdddot, qddddot] = reference_traj(obj,t)
            % This function organizes the 3 times diff trajectory
            ind = obj.getIndex(t);
            q = [obj.x(ind); obj.y(ind)];
            qdot = [obj.xdot(ind); obj.ydot(ind)];
            qddot = [obj.xddot(ind); obj.yddot(ind)];
            qdddot = [obj.xdddot(ind); obj.ydddot(ind)];
            qddddot = [obj.xdddot(ind); obj.ydddot(ind)];
        end
        
        function ind = getIndex(obj, t)
            ind = round(t/obj.dt+1,0);
        end
        
        function yaw = getYaw(obj,t)
            ind = round(t/obj.dt+1,0);
            yaw = obj.psi(ind);
        end
        
        function [v, w] = getVelocities(obj,t)
            ind = round(t/obj.dt+1,0);
            v = obj.v(ind);
            w = obj.w(ind);
        end
        
        function x = stateAtTime(obj,t)
            ind = round(t/obj.dt+1,0);
            x = [obj.x(ind); obj.y(ind); obj.psi(ind); obj.v(ind); obj.w(ind)];
        end
        
        function xdot = xdotAtTime(obj,t)
            ind = round(t/obj.dt+1,0);
            xdot = [obj.v(ind)*cos(obj.psi(ind)); obj.v(ind)*sin(obj.psi(ind)); obj.w(ind); obj.a(ind); obj.alpha(ind)];
        end
    end
end
