classdef SUPathControl < Controller
    %SUPathControl Performs path control for a Simple Unicycle
    
    properties
        eps = 0.2  % The following distance for an epsilon trajectory
        K_point_ctrl % Feedback matrix for the point control
    end
    
    methods
        function obj = SUPathControl(vehicle)
            obj = obj@Controller(vehicle);
            
            % Calculate feedback matrix for epsilon point control
            A = zeros(2); 
            B = eye(2);
            Q = diag([1, 1]);
            R = diag([1, 1]);
            obj.K_point_ctrl = lqr(A, B, Q, R);
        end
        
        function u = calculateControl(t, x, q_des, qd_des, qdd_des)
            % Uses epsilon control to calculate the desired (v,w) pair
            
            % Get states
            x_pos = x(obj.vehicle.kinematics.x_ind);
            y_pos = x(obj.vehicle.kinematics.y_ind);
            th = x(obj.vehicle.kinematics.th_ind);
            c = cos(th);
            s = sin(th);
            
            % Form espilon variables
            R_e_inv = [1 0; 0 1/obj.eps] * [c s; -s c];
            
            % Calculate current values of espilon state
            q_eps = [x_pos; y_pos] + obj.eps * [c; s];
            
            % Calculate point control
            u_point = -obj.K_point_ctrl*(q_eps - q_des) + qd_des;
            
            % Calculate the control inputs
            u = R_e_inv*u_point;
        end
    end
end

