classdef ContinuousSteeringBicycleWithIntegral < ContinuousSteeringBicycle
    %SimpleBicycle Implements a unicycle with direct control over the
    %velocities
    
    properties
        w_d;
        w_int_ind = 6; % Integral of omega
    end
    
    methods
        function obj = ContinuousSteeringBicycleWithIntegral(w_d)
            obj = obj@ContinuousSteeringBicycle();
            obj.dimensions = 6;
            obj.w_d = w_d;
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
            xdot(obj.phi_ind) = u_phi; % \dot{phi}
            
            % Integrate the error
            xdot(obj.w_int_ind) = (w - obj.w_d);
        end 
        
        
        
    end 
end

