classdef SimpleUnicycle < VehicleKinematics
    %SimpleUnicycle Implements a unicycle with direct control over the
    %velocities
    methods
        function obj = SimpleUnicycle()
            obj = obj@VehicleKinematics(3);
            obj.plot_path = true;
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the unicycle dynamics
            %   [x; y; theta] = [vcos(theta); vsin(theta); omega]
            %   u = [v; omega]
            
            % Extract inputs
            v = u(1); % Translational velocity
            w = u(2); % Rotational velocity
            
            % Calculate dynamics
            theta = x(obj.th_ind);  % Orientation
            xdot = zeros(3,1);
            xdot(obj.x_ind) = v * cos(theta); % \dot{x}
            xdot(obj.y_ind) = v * sin(theta); % \dot{y}
            xdot(obj.th_ind) = w; % \dot{theta}          
            
        end    
        
        function [v, w] = getVelocities(obj, t, x, u)
            v = u(1);
            w = u(2);
        end
    end 
end

