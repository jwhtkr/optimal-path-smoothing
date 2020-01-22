classdef SUVelocityVectorFieldControl < Controller
    %SUVelocityVectorFieldControl Implements a velocity-based vector field
    %controller for a Simple Unicycle vehicle
    
    properties
        vd_field_max = 5 % max velocity
        k_wd = 2; % Gain for the desired rotational velocity
    end
    
    methods
        function obj = SUVelocityVectorFieldControl(vehicle)
            obj = obj@Controller(vehicle);
        end
        
        function u = calculateControl(t, x, g_func)
            % Inputs:
            %   t: time
            %   x: state for a unicycle
            %   g_func: function handle for a vector field g(t, q, th)
            %       t: time
            %       q: 2D position
            %       th: orientation
            
            % Calculate the current vector
            g = g_func(t, x(obj.vehicle.q_ind), x(obj.vehicle.th_ind));
            
            % Calculate the desired velocity
            vd = norm(g);
            vd = min(vd, obj.vd_field_max); % Threshold the desired velocity
            
            % Calculate the desired orientation
            th_d = atan2(g(2), g(1));
            
            % Calculate the error in orientation
            th = x(obj.vehicle.th_ind);
            th_e = th-th_d;
            th_e = atan2(sin(th_e), cos(th_e)); % Adjust to ensure between -pi and pi
            
            % Calculate the desired rotational velocity
            wd = -obj.k_wd*th_e;
            
            % Use velocity control to follow the vector field
            u = [vd; wd];
        end
    end
end

