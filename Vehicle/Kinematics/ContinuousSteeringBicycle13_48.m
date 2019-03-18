classdef ContinuousSteeringBicycle13_48 < SimpleBicycle
    %SimpleBicycle Implements a unicycle with direct control over the
    %velocities
    
    properties
        v_ind = 4;
        phi_ind = 5
        phi_dot_ind = 6;
    end
    
    methods
        function obj = ContinuousSteeringBicycle13_48()
            obj = obj@SimpleBicycle();
            obj.dimensions = 6;
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the unicycle dynamics
            %   [x; y; theta] = [vcos(theta); vsin(theta); omega]
            %   u = [v; phi]
            
            % Extract inputs
            u_v = u(1);
            u_phi = u(2);
            
            % Extract velocity and turning angle
            v = x(obj.v_ind);
            phi = x(obj.phi_ind);
            
            % Calculate rotational velocity
            w = v/obj.L * tan(phi); % Rotational velocity
            
            % Calculate dynamics
            theta = x(obj.th_ind);  % Orientation
            xdot = zeros(obj.dimensions,1);
            xdot(obj.x_ind) = v * cos(theta); % \dot{x}
            xdot(obj.y_ind) = v * sin(theta); % \dot{y}
            xdot(obj.th_ind) = w; % \dot{theta} 
            xdot(obj.v_ind) = u_v; % \dot{v}
            xdot(obj.phi_ind) = x(obj.phi_dot_ind); % \dot{phi}
            xdot(obj.phi_dot_ind) = u_phi; % \dot{phi}
        end 
        
        function [v, w] = getVelocities(obj, t, x, u)
            % Extract velocity and turning angle
            v = x(obj.v_ind);
            phi = x(obj.phi_ind);
            
            % Calculate rotational velocity
            w = v/obj.L * tan(phi); % Rotational velocity
        end
        
    end 
end

