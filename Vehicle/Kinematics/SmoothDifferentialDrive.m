classdef SmoothDifferentialDrive < DifferentialDrive
    %SimpleUnicycle Implements a unicycle with direct control over the
    %velocities
    
    properties
        ind_wr = 4 % Rotational velocity of right wheel
        ind_wl = 5 % Rotational velocity of left wheel
    end
    
    methods
        function obj = SmoothDifferentialDrive()
            obj = obj@DifferentialDrive();
            obj.dimensions = 5;
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the unicycle dynamics
            %   [x; y; theta] = [vcos(theta); vsin(theta); omega]
            %   u = [v; omega]
            
            % Extract inputs
            ur = u(1); % Rotational velocity of right wheel
            ul = u(2); % Rotational velocity of left wheel
            
            % Extract wheel angular velocities
            wr = x(obj.ind_wr);
            wl = x(obj.ind_wl);
            
            % Calculate velocities
            v = obj.rad/2*(wr+wl); % Translational velocity
            w = obj.rad/obj.L*(wr-wl); % Rotational velocity
%             if t >= 8.33
%                 wr = wr + 0.000001;
%             end
            % Calculate dynamics
            theta = x(obj.th_ind);  % Orientation
            xdot = zeros(obj.dimensions,1);
            xdot(obj.x_ind) = v * cos(theta); % \dot{x}
            xdot(obj.y_ind) = v * sin(theta); % \dot{y}
            xdot(obj.th_ind) = w; % \dot{theta} 
            xdot(obj.ind_wr) = ur; % \dot{wr}
            xdot(obj.ind_wl) = ul; % \dot{wl}            
        end   
        
        function [v, w] = getVelocities(obj, t, x, u)
            % Extract wheel angular velocities
            wr = x(obj.ind_wr);
            wl = x(obj.ind_wl);
            
            % Calculate velocities
            v = obj.rad/2*(wr+wl); % Translational velocity
            w = obj.rad/obj.L*(wr-wl); % Rotational velocity
        end
    end 
end

