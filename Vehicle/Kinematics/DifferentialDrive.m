classdef DifferentialDrive < VehicleKinematics
    %SimpleUnicycle Implements a unicycle with direct control over the
    %velocities
    
    properties
        rad = 0.25;
        L = 1;
    end
    
    methods
        function obj = DifferentialDrive()
            obj = obj@VehicleKinematics(3);
            obj.plot_path = true;
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the unicycle dynamics
            %   [x; y; theta] = [vcos(theta); vsin(theta); omega]
            %   u = [v; omega]
            
            % Extract inputs
            ur = u(1); % Rotational velocity of right wheel
            ul = u(2); % Rotational velocity of left wheel
            
            % Calculate velocities
            v = obj.rad/2*(ur+ul); % Translational velocity
            w = obj.rad/obj.L*(ur-ul); % Rotational velocity
            
            % Calculate dynamics
            theta = x(obj.th_ind);  % Orientation
            xdot = zeros(3,1);
            xdot(obj.x_ind) = v * cos(theta); % \dot{x}
            xdot(obj.y_ind) = v * sin(theta); % \dot{y}
            xdot(obj.th_ind) = w; % \dot{theta}          
            
        end  
        
        function [v, w] = getVelocities(obj, t, x, u)
            % Extract inputs
            ur = u(1); % Rotational velocity of right wheel
            ul = u(2); % Rotational velocity of left wheel
            
            % Calculate velocities
            v = obj.rad/2*(ur+ul); % Translational velocity
            w = obj.rad/obj.L*(ur-ul); % Rotational velocity
        end
    end 
end

