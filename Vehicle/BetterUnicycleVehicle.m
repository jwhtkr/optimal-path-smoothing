classdef BetterUnicycleVehicle < Vehicle
   
    properties
        % Properties for velocity control
        
        % Properties for path control (using point control method)
        eps_path = 1.0 % Initial epsilon for controlling a point to a path
        eps_path_min = 0.2 % Minimum value for eps_path
        K_point_ctrl % Feedback matrix for point control, used with feedback on 
                     % K_point_ctrl*(q - q_des), where q is the
                     % position and velocity of a point
                     
                     
        % Properties for velocity control 
        K_vel % Feedback matrix for velocity control where the state is the 
              % translational and rotational velocities
    end
    
    methods
        function obj = BetterUnicycleVehicle(varargin)
            % Get the initial state
            x0 = [0 0 0 0 0]'; % default to the zero state
            if nargin > 0
                x0 = varargin{1}; 
            end
            
            % Initialize the kinematics and the vehicle
            kin = BetterUnicycle;
            q_ind = [kin.x_ind; kin.y_ind];
            obj = obj@Vehicle(kin, x0, q_ind);  
            
            
            % Calculate feedback matrix for point control
            A = [zeros(2) eye(2); zeros(2,4)]; 
            B = [zeros(2); eye(2)];
            Q = diag([1, 1, 1, 1]);
            R = diag([1, 1]);
            obj.K_point_ctrl = lqr(A, B, Q, R);
            
            % Calculate feedback matrix for velocity control
            A = zeros(2);
            B = eye(2);
            Q = diag([1, 1]);
            R = diag([1, 1]);
            obj.K_vel = lqr(A, B, Q, R);
        end
        
        function u = velocityControl(obj, vd, wd, varargin)
            % Get the state
            if nargin > 5
                x = varargin{1};
            else
                x = obj.x;
            end
            
            % Extract states
            v = x(obj.kinematics.v_ind);
            w = x(obj.kinematics.w_ind);
            
            % Calculate new state
            z = [v - vd; w - wd;];
            
            % Calculate control
            u = -obj.K_vel * z;
        end
        
        function u = pathControl(obj, t, q_des, qd_des, qdd_des, varargin)
            %pathControl will calculate the desired control to track a path
            %  defined by the inputs:
            %   t: Time
            %   q_des: Desired position
            %   qd_des: Desired velocity vector of a point mass
            %   qdd_des: Desired acceleration vector of a point mass
            %
            %  With the output
            %    u: control input to the system (u_v, u_omega)
            %
            % Note that the current time t is used to calcualte the value
            % for epsilon in the epsilon point control
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Get the state
            if nargin > 5
                x = varargin{1};
            else
                x = obj.x;
            end
            
            % Calculate epsilon
            eps = obj.getEpsilon(t);
            
            % Get states
            x_pos = x(obj.kinematics.x_ind);
            y_pos = x(obj.kinematics.y_ind);
            [v, w] = obj.kinematics.getVelocities(t, x, 0);
            th = x(obj.kinematics.th_ind);
            c = cos(th);
            s = sin(th);
            
            % Form espilon variables
            w_hat_e = [0 -eps*w; w/eps 0];
            R_e = [c -eps*s; s eps*c];
            R_e_inv = [1 0; 0 1/eps] * [c s; -s c];
            
            % Calculate current values of espilon state
            q_eps = [x_pos; y_pos] + eps * [c; s];
            q_eps_dot = R_e*[v; w];
            q = [q_eps; q_eps_dot];
            
            % Calculate point control
            u_point = -obj.K_point_ctrl*(q - [q_des; qd_des]) + qdd_des;
            
            % Calculate the control inputs
            u = R_e_inv*u_point - w_hat_e*[v; w];            
        end
        
    end
    
    methods (Access=protected)
        function eps = getEpsilon(obj, t)
            eps = obj.eps_path*exp(-t);
            eps = max(obj.eps_path_min, eps);
        end
    end
end

