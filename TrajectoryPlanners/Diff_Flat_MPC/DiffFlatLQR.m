classdef DiffFlatLQR < handle
    %DiffFlatLQR implments a time varying control law to follow a
    %trajectory produced by a differentially flat input
    
    properties(Access=protected)
        df_traj = ExactTrajRep4(); % Stores the differentially flat trajectory
        T = 15; % Final time for control
        P_0 % Stores the initial value for the DRE
        
        % Cost values for LQR problem
        Q = 0.1 .* eye(5);
        R = 0.1 .* eye(2);
        R_inv % Inverse of R
        S = diag([1, 1, 0, 0, 0]);
        
        % State matrices
        B = [zeros(3,2); eye(2)]; % Linearized input matrix
        BRB % B(t) R B^T(t)
        A_const
        n_x = 5;
        n_u = 2;
    end
    
    properties(Constant)
        x_ind = 1;
        y_ind = 2;
        psi_ind = 3;
        v_ind = 4;
        w_ind = 5;
    end
    
    methods
        function obj = DiffFlatLQR()
            %DiffFlatLQR Construct an instance of this class            
            obj.R_inv = inv(obj.R);
            obj.A_const = zeros(obj.n_x);
            obj.A_const(3,1) = 1;
            obj.BRB = obj.B * obj.R_inv * obj.B';
        end
        
        function setValues(obj, x_in, u_in, dt, t1)
        %setValues Stores the values of the differentially flat system and
        %updates the DRE results
        %
        % Inputs:
        %   x_in: states, can be either a column vector or 8x(N+1) matrix
        %   u_in: controls, can be either a column vector a 2xN matrix
        %   dt: time spacing
        %   t1: time of x(1)
            % Store the inputs
            obj.df_traj.setValues(x_in, u_in, dt, t1);
        end
        
        function u = calculateControl(obj, t, x_veh)
        %calculateControl: Calculates the control at time t given the
        %vehicle state in x_veh
        
            % Calculate the DRE at time t
            P = obj.calculatePLinSys(t);
            
            % Calculate the desired input and control
            [xd, ud] = obj.calculateDesiredStateInput(t);
            
            % Calculate the state difference
            dx = x_veh - xd;
            
            % Update the orientation variable to be between -pi and pi
            dpsi = dx(obj.psi_ind);
            dpsi = atan2(sin(dpsi), cos(dpsi));
            dx(obj.psi_ind) = dpsi;
            
            % Calculate the control
            du = -obj.R_inv*obj.B'*P*dx;
            u = ud + du;
        end
        
        function P = calculatePLinSys(obj, t)
        %calculatePLinSys Calculates the solution to the DRE at time t <
        %obj.T using a combination of linear systems
            % Use Euler integration to integrate backwards in time
            X = eye(obj.n_x);
            Y = obj.S;
            dt = 0.001;
            
            % Start from the terminal time and integrate backwards
            t_sim = obj.T + obj.df_traj.t1;
%             p_vec = reshape(Y, [], 1);
%             t_vec = t_sim;
            tic
            while t_sim > t
                % Get the state matrix
                [x_diff_flat, ~] = obj.df_traj.getStateAndControl(t_sim);
                A = obj.calculateLinearizedStateMatrix(x_diff_flat);
                
                % Calculate the time derivatives
                Xdot = A*X - obj.BRB*Y;
                Ydot = -obj.Q*X - A'*Y;
                
                % Update the state
                X = X - dt * Xdot;
                Y = Y - dt * Ydot;
                t_sim = t_sim - dt;   
                
%                 % Store the value of P
%                 Ptmp = Y*inv(X);
%                 p_vec = [p_vec reshape(Ptmp, [], 1)];
%                 t_vec = [t_vec t_sim];
            end
            
            % Calculate the P matrix
            P = Y/X;
            time_lin = toc
            
%             % Calculate using exponential (assuming constant A)
%             M = [A -obj.B*obj.R_inv*obj.B'; -obj.Q -A'];
%             X0 = eye(5);
%             Y0 = obj.S;
%             agg_mat = expm(M*(-obj.T))*[X0; Y0];
%             X = agg_mat(1:5, :);
%             Y = agg_mat(6:10, :);
%             
%             P_calc = Y/X;
%             
%             err = norm(P-P_calc, 'fro')
            
            % Calculate via ode45
%             tic
%             [Pode, pvec_tmp, tvec_tmp] = obj.calculatePOde(t);
%             time_ode = toc
%             err = norm(P-Pode, 'fro')
%             P = Pode;
            
%             figure('units','normalized','outerposition',[0 0 1 1]);
%             for k = 1:size(p_vec, 1)
%                 subplot(5,5,k);
%                 plot(tvec_tmp, pvec_tmp(k,:), 'r', 'linewidth', 3); hold on;
%                 plot(t_vec, p_vec(k,:), 'b', 'linewidth', 2);
%                 ylabel(['p' num2str(k)]);
%             end
        end
    end
    
    methods(Access=protected)
        function [Pout, pvec, t_mat] = calculatePOde(obj, t)
        % This fucntion calculates the P matrix using ODE45, but it is very
        % prone to error
            % Initialize solution
            PT = obj.S;
            pT = reshape(PT, [], 1);
            t_vec = [obj.T+obj.df_traj.t1:-.0001:t];
            
            % Simulate backwards in time
            %[t_mat, p_mat] = ode45(@(t_val, p_val)dynamicsP(t_val, p_val, obj), t_vec, pT);
            [t_mat, p_mat] = ode15s(@(t_val, p_val)dynamicsP(t_val, p_val, obj), t_vec, pT);
            
            % Extract the initial P
            p = p_mat(end,:)';
            Pout = reshape(p, obj.n_x, obj.n_x);
            
            pvec = p_mat';
            
            function pdot = dynamicsP(t_val, p_val, obj)
                % Create the P matrix
                P = reshape(p_val, obj.n_x, obj.n_x);
                
                % Get the state matrix
                [x_diff_flat, ~] = obj.df_traj.getStateAndControl(t_val);
                A = obj.calculateLinearizedStateMatrix(x_diff_flat);
                
                % Get the time derivative of the DRE
                Pdot = -A'*P - P*A - obj.Q + P*obj.B*obj.R_inv*obj.B'*P;
                
                % Output the vector form
                pdot = reshape(Pdot, [], 1);
            end
        end
        
        function A = calculateLinearizedStateMatrix(obj, x_diff_flat)
        %calculateLinearizedStateMatrix calculates the time varying state matrix
        %from the differentially flat input
        %
        % Inputs:
        %   x_diff_flat = [q; q_dot; q_ddot; q_dddot]

            % Extract needed differentially flat states
            qdot = x_diff_flat(3:4);

            % Calculate the desired vehicle parameters
            psi = atan2(qdot(2), qdot(1));
            v = norm(qdot);

            % Calculate the state matrix
            s = sin(psi);
            c = cos(psi);
            A = obj.A_const;
            A(1,3) = -v*s; % dx/dpsi
            A(1,4) = c; % dx/dv
            A(2,3) = v*c; % dy/dpsi
            A(2,4) = s; % dy/dv
        end
        
        function [xd, ud] = calculateDesiredStateInput(obj, t)
        %calculateDesiredStateInput calculates the desired state and input
        %at time t
            % Get the differentially flat state at time t
            [x_diff_flat, ~] = obj.df_traj.getStateAndControl(t);
            traj.q = x_diff_flat(1:2);
            traj.qdot = x_diff_flat(3:4);
            traj.qddot = x_diff_flat(5:6);
            traj.qdddot = x_diff_flat(7:8);
            
            % Calculate the desired trajectory information
            [psi, v, w, a, alpha] = TrajUtil.getTrajectoryInformation(traj);
            
            % Calculate the desired state and input
            xd = [traj.q; psi; v; w];
            ud = [a; alpha];
        end
    end
end



