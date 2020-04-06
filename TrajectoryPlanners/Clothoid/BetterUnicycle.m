classdef BetterUnicycle < VehicleKinematics
    %BetterUnicycle Implements a unicycle with direct control over the
    %accleration
    
    properties
       v_ind = 4
       w_ind = 5
    end
        
    methods
        function obj = BetterUnicycle()
            obj = obj@VehicleKinematics(5);
            obj.plot_path = true;
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the unicycle dynamics
            %   [x; y; theta] = [vcos(theta); vsin(theta); omega]
            %   u = [v; omega]
            
            % Extract inputs
            u_v = u(1); % Translational velocity
            u_w = u(2); % Rotational velocity
            
            % Extract states
            v = x(obj.v_ind);
            w = x(obj.w_ind);
            
            % Calculate dynamics
            theta = x(obj.th_ind);  % Orientation
            xdot = zeros(obj.dimensions,1);
            xdot(obj.x_ind) = v * cos(theta); % \dot{x}
            xdot(obj.y_ind) = v * sin(theta); % \dot{y}
            xdot(obj.th_ind) = w; % \dot{theta}            
            xdot(obj.v_ind) = u_v;
            xdot(obj.w_ind) = u_w;
        end   
        
        function [v, w] = getVelocities(obj, t, x, u)
            v = x(obj.v_ind);
            w = x(obj.w_ind);
        end
    end 
end

