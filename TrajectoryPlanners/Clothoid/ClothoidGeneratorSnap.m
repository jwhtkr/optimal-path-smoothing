classdef ClothoidGeneratorSnap < handle
    %BetterUnicycle Implements a unicycle with direct control over the
    %accleration
    
    properties(Access = protected)
       dt;
       v;
       max_k;
       max_sigma; 
       
       
       clothoid;
       t_span;
       
       circle;
       mu;
       waypoints;
       waypoints_directon;
       max_clothoid_deflection;
       
       max_sig_accel % Maximum acceleration of sigma
       
       curvature % Instance of the SmoothCurvature object
    end
    
    properties
        traj;
    end
    
    % State variables
    properties (SetAccess=protected)
        q1_ind = 1 % Index of the first position
        q2_ind = 2 % Index of the second position
        psi_ind = 3 % Index of the orientation
        k_ind = 4; % Index of the curvature
        s_ind = 5; % Index of the curvature change rate (first derivative)
        g_ind = 6; % Index of the second derivative of curvature
        
        n_states = 6; % Number of total states
        n_states_wo_curvature = 3; % Number of states not including the curvature states
    end
        
    methods
        function obj = ClothoidGeneratorSnap(max_k, v, dt, max_sigma, varargin)
            % Inputs:
            %   max_k: maximum curvature
            %   v: velocity
            %   dt: time step
            %   max_sigma: maximum change in curvature
            %   varargin{1}: maximum second derivative of sigma
            
            % Store input variables
            obj.dt = dt;  % time step
            obj.v = v; % velocity
            obj.max_k = max_k; % maximum curvature
            obj.max_sigma = max_sigma; % maximum change in curvature            
            
            % Extract maximum second derivative of sigma
            if length(varargin) > 0
                obj.max_sig_accel = varargin{1};                
            else
                obj.max_sig_accel = 100;
            end
            
            % Calculate the full clothoid
            obj.curvature = SmoothCurvature(obj.max_sig_accel, obj.max_sigma, obj.max_k);
            obj.t_span = obj.curvature.getCurvatureTimeSpan(obj.dt);
            x = obj.calcClothoid();
            
            % Set the trajectory variables
            obj.storeTrajectoryData(x);
        end
        
        function clothoid = calcClothoid(obj)
            x0 = zeros(obj.n_states_wo_curvature,1);
            
            % Integrate the bicycle dynamics
            [t_vec,x_vec] = ode45(@(t,x) obj.xdot(t, x), obj.t_span, x0);

            % Get the curvature states
            x_k = obj.curvature.calculateClothoidCurvature(t_vec);
            
            clothoid = [x_vec'; x_k];
        end
        
        function storeTrajectoryData(obj, x)
            % Initialize the trajectory
            obj.traj = Trajectory2D();
            obj.traj.ds = obj.v*obj.dt;
            s_len = obj.v*obj.curvature.t_kappa_max;
            obj.traj.s = 0:obj.traj.ds:s_len;
            obj.traj.cloth_len = length(obj.t_span);
            obj.traj.sigma = x(obj.s_ind,:);
            obj.traj.dt = obj.dt;
            obj.traj.t = obj.t_span;
            obj.traj.s_geo = obj.traj.s;
            
            % Initialize derivative storage
            len = size(x, 2);
            qdot_mat = zeros(2, len);
            qddot_mat = zeros(2, len);
            q_3_mat = zeros(2, len);
            q_4_mat = zeros(2, len);
            J = [0 -1; 1 0];
            
            % Calculate the derivatives over time
            for k = 1:len
                % Extract states
                psi = x(obj.psi_ind, k);
                kappa = x(obj.k_ind, k);
                sigma = x(obj.s_ind, k);
                gamma = x(obj.g_ind, k);
                
                % Create orientation vector
                h = [cos(psi); sin(psi)];
                
                % Calcualte first derivative
                qdot_mat(:,k) = obj.v*h;
                
                % Calculate the second derivative
                qddot_mat(:,k) = obj.v^2*kappa*J*h;
                
                % Calculate the third derivative
                q_3_mat(:,k) = obj.v^2*sigma*J*h - obj.v^3*kappa^2*h;
                
                % Calculate the fourth derivative
                q_4_mat(:,k) = obj.v^2*gamma*J*h - 3*obj.v^3*kappa*sigma*h - ...
                    obj.v^4*kappa^3*J*h;                
            end
            
            % Extract state data
            obj.traj.psi = x(obj.psi_ind, :); % Orientation
            obj.traj.k = x(obj.k_ind, :);
            obj.traj.x = x(obj.q1_ind, :); % Position data
            obj.traj.y = x(obj.q2_ind, :);
            obj.traj.xdot = qdot_mat(1,:); % Velocity vector data
            obj.traj.ydot = qdot_mat(2,:);
            obj.traj.xddot = qddot_mat(1,:); % Acceleration vector data
            obj.traj.yddot = qddot_mat(2,:);
            obj.traj.xdddot = q_3_mat(1,:); % Jerk vector data
            obj.traj.ydddot = q_3_mat(2,:);
            obj.traj.xddddot = q_4_mat(1,:); % Snap vector data
            obj.traj.yddddot = q_4_mat(2,:);
            
            % Store translational data
            obj.traj.v = obj.v.*ones(1,len); % velocity
            obj.traj.a = zeros(1,len); % acceleration
            obj.traj.j = zeros(1,len); % jerk
            
            % Store rotational data
            obj.traj.w = obj.v.*x(obj.k_ind); % velocity
            obj.traj.alpha = obj.v.*x(obj.s_ind); % acceleration
            obj.traj.zeta = obj.v.*x(obj.g_ind); % jerk
        end
    end
    
    methods(Access=protected)
        function x_dot = xdot(obj, t, x)
            % Extract the orientation
            psi = x(obj.psi_ind);
            
            % Calculate the curvature
            kappa = obj.curvature.getCurvatureState(t);
            
            % Calcualte the time derivative of all the states
            x_dot = zeros(obj.n_states_wo_curvature, 1);
            x_dot(obj.q1_ind) = obj.v*cos(psi);
            x_dot(obj.q2_ind) = obj.v*sin(psi);
            x_dot(obj.psi_ind) = obj.v*kappa;
        end
    end
end
