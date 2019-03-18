classdef JerkUnicycle < VehicleKinematics
    %BetterUnicycle Implements a unicycle with direct control over the
    %accleration
    
    properties
       v_ind = 4 % Translational velocity state
       w_ind = 5 % Rotational velocity state
       a_ind = 6 % Translational acceleration state
       alpha_ind = 7 % Rotational acceleration state
    end
        
    methods
        function obj = JerkUnicycle()
            obj = obj@VehicleKinematics(7);
            obj.plot_path = true;
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the unicycle dynamics
            %   [x; y; theta] = [vcos(theta); vsin(theta); omega]
            %   u = [u_a; u_alpha]
            
            % Extract inputs
            u_a = u(1); % Translational velocity
            u_alpha = u(2); % Rotational velocity
            
            % Extract states
            v = x(obj.v_ind);
            a = x(obj.a_ind);
            w = x(obj.w_ind);
            alpha = x(obj.alpha_ind);            
            
            % Calculate dynamics
            theta = x(obj.th_ind);  % Orientation
            xdot = zeros(obj.dimensions,1);
            xdot(obj.x_ind) = v * cos(theta); % \dot{x}
            xdot(obj.y_ind) = v * sin(theta); % \dot{y}
            xdot(obj.th_ind) = w; % \dot{theta}            
            xdot(obj.v_ind) = a;
            xdot(obj.w_ind) = alpha;
            xdot(obj.a_ind) = u_a;
            xdot(obj.alpha_ind) = u_alpha;
        end   
        
        function [v, w] = getVelocities(obj, t, x, u)
            v = x(obj.v_ind);
            w = x(obj.w_ind);
        end
    end 
end

