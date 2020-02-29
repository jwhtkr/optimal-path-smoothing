classdef SmoothDifferentialDriveVehicle < Vehicle
   
    properties
        % Properties for path control (using point control method)
        eps_path = 1.0 % Initial epsilon for controlling a point to a path
        eps_path_min = 0.2 % Minimum value for eps_path
        use_dim_eps = false; % true => use a diminishing epsilon, false will use a constant epsilon of eps_path
        K_point_ctrl % Feedback matrix for point control, used with feedback on 
                     % K_point_ctrl*(q - q_des), where q is the
                     % position and velocity of a point
                     
        % Properties for point-method velocity control
        eps_vel = 0.2; % Epsilon to be used for velocity control
        K_point_vel % Feedback matrix for point control of velocity, used with feedback on 
                     % K_point_vel*(q - q_des), where q is the velocity of a point                     
                     
        % Properties for velocity control 
        K_vel % Feedback matrix for velocity control where the state is the 
              % translational and rotational velocities
              
              
        
        % Properties for vector field orientation control 
        K_orient % Feedback matrix for velocity control where the state is the 
              % translational and rotational velocities
              
        % Gains for control
        k_wd = 2; % Gain for the desired rotational velocity
        vd_field_max = 1; % Maximum desired velocity from a vector field
        
        % Point control - Mapping accelerations to wheel control
        map_accel_to_wheel; % Used to map the accelerations to the wheel control
    end
    
    methods
        function obj = SmoothDifferentialDriveVehicle(varargin)
            % Get the initial state
            x0 = [0 0 0 0 0]'; % default to the zero state
            if nargin > 0
                x0 = varargin{1}; 
            end
            
            % Initialize the kinematics and the vehicle
            kin = SmoothDifferentialDrive;
            q_ind = [kin.x_ind; kin.y_ind];
            obj = obj@Vehicle(kin, x0, q_ind);  
            
            
            % Calculate feedback matrix for point control
            A = [zeros(2) eye(2); zeros(2,4)]; 
            B = [zeros(2); eye(2)];
            Q = diag([1, 1, 1, 1]);
            R = diag([1, 1]);
            obj.K_point_ctrl = lqr(A, B, Q, R);
            
            % Calculate feedback matrix for velocity point control
            A = zeros(2);
            B = eye(2);
            Q = diag([10, 10]);
            R = diag([.1, .1]);
            obj.K_point_vel = lqr(A, B, Q, R);
            
            % Calculate feedback matrix for velocity control
            r = kin.rad;
            L = kin.L;
            A = zeros(2);
            B = [r/2 r/2; r/L -r/L];
            Q = diag([100, 100]);
            R = diag([1, 1]);
            obj.K_vel = lqr(A, B, Q, R);
            
            % Create mapping from accelerations to wheel control
            obj.map_accel_to_wheel = inv( B );
            
            % Calculate feedback matrix for orientation control
            A = [0 0 0; 0 0 1; 0 0 0];
            B = [r/2 r/2; 0 0; r/L -r/L];
            Q = diag([1 10 0]);
            R = diag([.1 .001]);
            obj.K_orient = lqr(A, B, Q, R);
            
           
        end
        
        function u = velocityControl(obj, vd, wd, varargin)
            % Get the state
            if nargin > 5
                x = varargin{1};
            else
                x = obj.x;
            end
            
            % Extract states
            [v, w] = obj.kinematics.getVelocities(0, x, 0); % zeros come from fact that velocities do not depend on time or input
            
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
            
            % Calculate the epsilon point control to follow the path
            u_eps = obj.epsilonPathControl(t, q_des, qd_des, qdd_des, x);
            
            % Calculate the vehicle control
            u = obj.epsilonToVehicleControl(t, x, u_eps);
        end
        
        function u_eps = epsilonPathControl(obj, t, q_des, qd_des, qdd_des, x)
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
            R_e = [c -eps*s; s eps*c];
            
            % Calculate current values of espilon state
            q_eps = [x_pos; y_pos] + eps * [c; s];
            q_eps_dot = R_e*[v; w];
            q = [q_eps; q_eps_dot];
            
            % Calculate point control
            u_eps = -obj.K_point_ctrl*(q - [q_des; qd_des]) + qdd_des;
        end
        
        function q_eps = calculateEpsilonPoint(obj, t, x)
            % Calculate epsilon
            eps = obj.getEpsilon(t);
            
            % Extract states
            x_pos = x(obj.kinematics.x_ind);
            y_pos = x(obj.kinematics.y_ind);
            th = x(obj.kinematics.th_ind);
            c = cos(th);
            s = sin(th);
            
            % Calculate the epsilon state
            q_eps = [x_pos; y_pos] + eps * [c; s];
        end
        
        function u = epsilonToVehicleControl(obj, t, x, u_eps)
            % Calculate epsilon
            eps = obj.getEpsilon(t);
            
            % Get states
            [v, w] = obj.kinematics.getVelocities(t, x, 0);
            th = x(obj.kinematics.th_ind);
            c = cos(th);
            s = sin(th);
            
            % Form espilon variables
            w_hat_e = [0 -eps*w; w/eps 0];
            R_e_inv = [1 0; 0 1/eps] * [c s; -s c];
            
            % Calculate the accelerations ([a; alpha])
            accel = R_e_inv*u_eps - w_hat_e*[v; w]; 
            
            % Calculate the control inputs
            u = obj.map_accel_to_wheel * accel;            
        end
        
        function u = pathVectorControl(obj, t, q_des, qd_des, qdd_des, g_function, varargin)
            %pathVectorControl combines control laws for path tracking and
            %vector field following. Both act as forces on a vehicle
            %
            %Inputs:
            %   t: Time
            %   q_des: Desired position
            %   qd_des: Desired velocity vector of a point mass
            %   qdd_des: Desired acceleration vector of a point mass
            %   g_function: function handle for obtaining vector.
            %               g_function is a function of (t, x)
            %
            %  With the output
            %    u: control input to the system (u_v, u_omega)
            %
            % Note that the current time t is used to calcualte the value
            % for epsilon in the epsilon point control
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Get the state
            if nargin > 6
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
            
            % Calculate the vector field for the espilon point
            g = g_function(t, q_eps, th);
            
            % Calculate point control for path following
            u_point = -obj.K_point_ctrl*(q - [q_des; qd_des]) + qdd_des
            
            % Add in the force caused by the vector field
            %u_point = u_point + g;
            
            % Calculate the accelerations ([a; alpha])
            accel = R_e_inv*u_point - w_hat_e*[v; w]; 
            
            % Calculate the control inputs
            u = obj.map_accel_to_wheel * accel;
        end
        
        %%%%%%%%%%%%%%%  Vector Field Following Controls %%%%%%%%%%%%%%%%
        function u = vectorFieldControl(obj, t, g,control_type, varargin)
            %vectorFieldControl will calculate the desired control to follow 
            % a vector field with the following inputs:
            %   t: Time
            %   g: vector to be followed
            %   varargin{1}: the state
            %
            %  With the output
            %    u: control input to the system (u_v, u_omega)            %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Get the state
            if nargin > 3
                x = varargin{1};
            else
                x = obj.x;
            end
            
            if control_type == VECTOR_FOLLOWING_TYPE.VELOCITY
                u = obj.velocityVectorFieldControl(t, g, x);
            elseif control_type == VECTOR_FOLLOWING_TYPE.POINT
                u = obj.pointVelocityVectorFieldControl(t, g, x);
            elseif control_type == VECTOR_FOLLOWING_TYPE.ORIENTATION
                u = obj.orientationVectorFieldControl(t, g, x);
            else
                u = [0;0];
            end
        end
        
        function u = velocityVectorFieldControl(obj, t, g_function, x)
            %vectorFieldControl will calculate the desired control to follow 
            % a vector field with the following inputs:
            %   t: Time
            %   g_function: function handle for obtaining vector.
            %   g_function is a function of (t, x) and returns the vector
            %   x: the state
            %
            %  With the output
            %    u: control input to the system (u_v, u_omega)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Calculate the current vector
            g = g_function(t, x(obj.q_ind), x(obj.th_ind));
            
            % Calculate the desired velocity
            vd = norm(g);
            vd = min(vd, obj.vd_field_max); % Threshold the desired velocity
            
            % Calculate the desired orientation
            th_d = atan2(g(2), g(1));
            
            % Calculate the error in orientation
            th = x(obj.th_ind);
            th_e = th-th_d;
            th_e = atan2(sin(th_e), cos(th_e)); % Adjust to ensure between -pi and pi
            
            % Calculate the desired rotational velocity
            wd = -obj.k_wd*th_e;
            
            % Use velocity control to follow the vector field
            u = obj.velocityControl(vd, wd, x);
        end
        
        function u = pointVelocityVectorFieldControl(obj, t, g_function, x)
            %pointVelocityVectorFieldControl will calculate the desired control to follow 
            % a vector field with the following inputs (using point control):
            %   t: Time
            %   g_function: function handle for obtaining vector.
            %   g_function is a function of (t, x) and returns the vector
            %   x: the state
            %
            %  With the output
            %    u: control input to the system (u_v, u_omega)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Get necessary states
            [v, w] = obj.kinematics.getVelocities(t, x, 0);
            th = x(obj.th_ind);
            c = cos(th);
            s = sin(th);
            
            % Form espilon variables
            eps = obj.eps_vel;
            w_hat_e = [0 -eps*w; w/eps 0];
            R_e = [c -eps*s; s eps*c];
            R_e_inv = [1 0; 0 1/eps] * [c s; -s c];
            
            % Calculate current velocity of espilon state
            q_eps = x(obj.q_ind) + eps*[c; s];
            q_eps_dot = R_e*[v; w];
            
            % Calculate the vector field for the espilon point
            g = g_function(t, q_eps, th);
            
            % Restrict the velocity of the vector field
            v_g = norm(g);
            if v_g > obj.vd_field_max
                g = obj.vd_field_max/v_g * g;
            end
            
            % Calculate point control
            u_point = -obj.K_point_vel*(q_eps_dot - g);
            
            % Calculate the commanded acceleration values            
            accel = R_e_inv*u_point - w_hat_e*[v;w];            
            
            % Calculate the commanding wheel inputs
            u = obj.map_accel_to_wheel * accel;
        end
        
        function u = orientationVectorFieldControl(obj, t, g_function, x)
            %vectorFieldControl will calculate the desired control to follow 
            % a vector field with the following inputs:
            %   t: Time
            %   g_function: function handle for obtaining vector.
            %   g_function is a function of (t, x) and returns the vector
            %   x: the state
            %
            %  With the output
            %    u: control input to the system (u_v, u_omega)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Calculate the current vector
            g = g_function(t, x(obj.q_ind), x(obj.th_ind));
            
            % Calculate the desired velocity
            vd = norm(g);
            vd = min(vd, obj.vd_field_max); % Threshold the desired velocity
            
            % Calculate the desired orientation
            th_d = atan2(g(2), g(1));
            
            % Calculate the error in orientation
            th = x(obj.th_ind);
            th_e = th-th_d;
            th_e = atan2(sin(th_e), cos(th_e)); % Adjust to ensure between -pi and pi
            
            % Extract states
            [v, w] = obj.kinematics.getVelocities(0, x, 0); % zeros come from fact that velocities do not depend on time or input
            
            % Create the aggregate state
            z = [v-vd; th_e; w];
            
            % Calculate the control from the aggregate state
            u = -obj.K_orient*z;
        end
        
        
    end
    
    methods (Access=public)
        function eps = getEpsilon(obj, t)
            if obj.use_dim_eps
                eps = obj.eps_path*exp(-t);
                eps = max(obj.eps_path_min, eps);
            else
                eps = obj.eps_path;
            end
        end
    end
end

