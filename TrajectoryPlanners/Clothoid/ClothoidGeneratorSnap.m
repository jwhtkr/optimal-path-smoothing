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
                x0 = varargin{2};
                direction = varargin{3};
                obj.curvature = SmoothCurvature(obj.max_sig_accel, obj.max_sigma, obj.max_k, [direction*obj.max_k; 0; 0], direction);
                obj.t_span = obj.curvature.getCurvatureTimeSpan(obj.dt);
                x = obj.calcClothoidWithInitialState(x0, direction);
            else
                obj.max_sig_accel = 100;
                obj.curvature = SmoothCurvature(obj.max_sig_accel, obj.max_sigma, obj.max_k);
                obj.t_span = obj.curvature.getCurvatureTimeSpan(obj.dt);
                x = obj.calcClothoid();
            end
            
            % Calculate the full clothoid
%             obj.t_span = obj.curvature.getCurvatureTimeSpan(obj.dt);
%             x = obj.calcClothoid();
            
            % Set the trajectory variables
            obj.storeTrajectoryData(x);
        end
        
        function clothoid = calcClothoid(obj)
            x0 = zeros(obj.n_states_wo_curvature,1);
            
            % Integrate the bicycle dynamics
            [t_vec,x_vec] = ode45(@(t,x) obj.xdot(t, x, false, 1), obj.t_span, x0);

            % Get the curvature states
            x_k = obj.curvature.calculateClothoidCurvature(t_vec);
            
            clothoid = [x_vec'; x_k];
        end
        
        function clothoid = calcClothoidWithInitialState(obj,x0,direction)            
            % Integrate the bicycle dynamics
            [t_vec,x_vec] = ode45(@(t,x) obj.xdot(t, x, true, direction), obj.t_span, x0);

            % Get the curvature states
            x_k = obj.curvature.calculateClothoidCurvature(t_vec, true, direction);
            
            clothoid = [x_vec'; x_k];
        end
        
        
        function [t_vec, psi] = eulerIntegrateCheck(obj,x_k, t_vec, x_vec)
            psi = zeros(1, length(t_vec));
            psi(1) = 0;
            
            dt_new = 0.001;
            t_vec_new = 0:dt_new:t_vec(end);
            
            for k = 1:(length(t_vec_new)-1)
                t = t_vec_new(k);
                psi(k+1) = psi(k) + dt_new*obj.v*obj.curvature.getCurvatureState(t);
            end
            
            % plot a comparison
            figure;
            plot(t_vec_new, psi, 'b', 'linewidth', 4); hold on;
            plot(t_vec, x_vec(obj.psi_ind, :), 'r', 'linewidth', 2);
            
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
            obj.traj.w = obj.v.*x(obj.k_ind, :); % velocity
            obj.traj.alpha = obj.v.*x(obj.s_ind, :); % acceleration
            obj.traj.zeta = obj.v.*x(obj.g_ind, :); % jerk
        end
    end
    
    methods(Access=protected)
        function x_dot = xdot(obj, t, x, varargin)
            % Extract the orientation
            psi = x(obj.psi_ind);
            
            if ~isempty(varargin)
                % Calculate the curvature
                x0 = varargin{1};
                direction = varargin{2};
                kappa = obj.curvature.getCurvatureState(t,x0, direction);
            else
                % Calculate the curvature
                kappa = obj.curvature.getCurvatureState(t);
            end
            
            % Calcualte the time derivative of all the states
            x_dot = zeros(obj.n_states_wo_curvature, 1);
            x_dot(obj.q1_ind) = obj.v*cos(psi);
            x_dot(obj.q2_ind) = obj.v*sin(psi);
            x_dot(obj.psi_ind) = obj.v*kappa;
        end
        
        function eulerIntegrateTrajectory(obj)
            
            x0 = [obj.traj.x(1); obj.traj.xdot(1); obj.traj.xddot(1); obj.traj.xdddot(1)];
            u = obj.traj.xddddot;
            
            x_int = [x0];
            x = x0;
            

            A = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
            B = [0; 0; 0; 1];
            for k = 1:(length(u)-1)
                x = x + obj.dt*(A*x + B*u(k));
                x_int = [x_int x];
            end
            
            % Plot the states
            figure;
            subplot(5,1,1);
            plot(obj.traj.x, 'b'); hold on;
            plot(x_int(1,:), 'r');
            ylabel('x')

            subplot(5,1,2);
            plot(obj.traj.xdot, 'b', 'linewidth', 4); hold on;
            plot(x_int(2,:), 'r');
            ylabel('xdot')

            subplot(5,1,3);
            plot(obj.traj.xddot, 'b'); hold on;
            plot(x_int(3,:), 'r');
            ylabel('xddot')

            subplot(5,1,4);
            plot(obj.traj.xdddot, 'b'); hold on;
            plot(x_int(4,:), 'r');
            ylabel('xdddot')

            subplot(5,1,5);
            plot(obj.traj.xddddot, 'b'); hold on;
            plot(obj.traj.xddddot, 'r');
            ylabel('xddddot')
            
            
            
        end
        
        function checkTrajectoryGeneration(obj, x)
            % Create variables from trajectory data
            psi_vec = zeros(1, obj.traj.cloth_len);
            v_vec = zeros(1, obj.traj.cloth_len);
            w_vec = zeros(1, obj.traj.cloth_len);
            a_vec = zeros(1, obj.traj.cloth_len);
            alpha_vec = zeros(1, obj.traj.cloth_len);
            
            % Extract the data from the trajectory
            for k = 1:obj.traj.cloth_len
                traj_in.q = [obj.traj.x(k); obj.traj.y(k)];
                traj_in.qdot = [obj.traj.xdot(k); obj.traj.ydot(k)];
                traj_in.qddot = [obj.traj.xddot(k); obj.traj.yddot(k)];
                traj_in.qdddot = [obj.traj.xdddot(k); obj.traj.ydddot(k)];
                
                % Get data
                [psi_vec(k), v_vec(k), w_vec(k), a_vec(k), alpha_vec(k)] = ...
                    getTrajectoryInformation(traj_in);
            end
            
            % Plot the comparisons
            figure;
            subplot(5,1,1);
            plot(psi_vec, 'b', 'linewidth', 3); hold on;
            plot(x(obj.psi_ind, :), 'r', 'linewidth', 2);
            
            subplot(5,1,2);
            plot(v_vec, 'b', 'linewidth', 3); hold on;
            plot(obj.v*ones(1, obj.traj.cloth_len), 'r', 'linewidth', 2);
            
            subplot(5,1,3);
            plot(w_vec, 'b', 'linewidth', 3); hold on;
            plot(obj.v*x(obj.k_ind, :), 'r', 'linewidth', 2);
            
            subplot(5,1,4);
            plot(a_vec, 'b', 'linewidth', 3); hold on;
            plot(zeros(1, obj.traj.cloth_len), 'r', 'linewidth', 2);
            
            subplot(5,1,5);
            plot(alpha_vec, 'b', 'linewidth', 3); hold on;
            plot(obj.v*x(obj.s_ind, :), 'r', 'linewidth', 2);
            
            % Plot the calculated values
            figure;
            subplot(2,1,1);
            k = w_vec ./ v_vec;
            plot(k, 'b', 'linewidth', 3); hold on;
            plot(x(obj.k_ind, :), 'r', 'linewidth', 2);
            ylabel('kappa');
            
            subplot(2,1,2);
            s = alpha_vec./v_vec - w_vec./(v_vec.^2) .* a_vec;
            plot(s, 'b', 'linewidth', 3); hold on;
            plot(x(obj.s_ind, :), 'r', 'linewidth', 2);
            ylabel('sigma');
        end
    end
end

function [psi, v, w, a, alpha] = getTrajectoryInformation(traj)
%getTrajectoryInformation calcualte trajectory information directly from
%trajectory
%
% Inputs:
%   traj: Struct with trajectory information
%       .q = position
%       .qdot = velocity vector
%       .qddot = acceleration vector
%       .qdddot = jerk vector
%
% Outputs:
%   psi: orientation
%   v: translational velocity
%   w: rotational velocity
%   a: translational acceleration
%   alpha: rotational acceleration

    % Extract trajectory information
    xdot = traj.qdot(1); % Velocity vector
    ydot = traj.qdot(2);
    xddot = traj.qddot(1); % Accleration vector
    yddot = traj.qddot(2);
    xdddot = traj.qdddot(1); % Jerk vector
    ydddot = traj.qdddot(2);
    
    % Calculate the trajectgory variables
    psi = atan2(ydot, xdot);
    v = sqrt(xdot^2+ydot^2);
    w = 1/v^2*(xdot*yddot - ydot*xddot);
    a = (xdot*xddot + ydot*yddot)/v;
    alpha = (xdot*ydddot-ydot*xdddot)/v^2 - 2*a*w/v;    
end
