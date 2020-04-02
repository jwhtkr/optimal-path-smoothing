classdef LinearSystemQuadraticCost < LinearSystemOptimization
    properties
        % Optimization weights
        R = 0.1 .* diag([1, 1]); % Input squared cost (i.e. u'*R*u)
        Q = 10 .* diag([1, 1,  0, 0,   0, 0, 0, 0]); % state error squared (x-x_d)'Q(x-x_d)
        S = [] % state error squared (x-x_d)'S(x-x_d)
        
        % Variable bounds
        u_max = [1; 1];
        x_max = [10; 10; 0.5; 0.5; 5; 5; 5; 5];
        x_min = [-1; -1; -0.5; -0.5; -5; -5; 5; 5];
%         x_max = inf.*ones(6, 1);
%         x_min = -inf.*ones(6, 1);
        
        % Desired variables
        xd = [] % Desired state over time
        ud = [] % Desired input over time
    end
    
    %%% Initialization functions %%%
    methods
        function obj = LinearSystemQuadraticCost(A, B, N, dt)
            %Construct an instance of this class
            %
            % Inputs:
            %   A: Continous-time state matrix
            %   B: Continuous-time input matrix
            
            obj = obj@LinearSystemOptimization(A, B, N, dt);
        end
        
        function obj = initializeParameters(obj)
        %initializeParameters Initializes the parameter optimization
        %parameters. In addition to the super class, this function creates
        %the terminal cost matrix used for optimization by selecting the
        %cost to be the infinite horizion cost associated with the infinite horizon
        %lqr problem
            % Call super class
            obj = initializeParameters@LinearSystemOptimization(obj);
            
            % Calculate the discrete-time algebraic riccati equation
            obj.S = dare(obj.Abar, obj.Bbar, obj.Q, obj.R);
            
            %% Define bounds
            % Define bounds on control
            lb_ctrl = repmat(-obj.u_max, obj.N, 1);
            ub_ctrl = repmat(obj.u_max, obj.N, 1);
            
            % Define bounds on state (only used right now for sequential
            % method)
            lb_state = repmat(obj.x_min, obj.N+1, 1);
            ub_state = repmat(obj.x_max, obj.N+1, 1);
            
            % Insert for sequential method
            obj.P_seq.lb = lb_ctrl;
            obj.P_seq.ub = ub_ctrl;
            
            % Insert for the simultaneous optimization method
            obj.P_simult.lb = [lb_state; lb_ctrl];
            obj.P_simult.ub = [ub_state; ub_ctrl];
            obj.P_simult.options.Display = 'notify-detailed';
        end
        
        function obj = updateSimBounds(obj, x_min, x_max, u_max)
        %updateSimBounds updates the bounds on the state and control (it is
        %assumed that the control bounds are symmetric)
        %
        % Inputs:
        %   x_min: 6x1 vector describing the lower bounds for the states
        %   x_max: 6x1 vector describing the upper bounds for the states
        %   u_max: 2x1 vector describing the upper bounds for the control
        %   and the lower bound using -u_max
            obj.x_min = x_min;
            obj.x_max = x_max;
            obj.u_max = u_max;
            obj = obj.initializeParameters();
        end
    end
    
    %%% Problem specific functions %%%
    methods
        function c = cost(obj, x, u)
        %cost calculates the cost given the state and inputs
        %
        % Inputs: 
        %   x: State (x_0 to x_N in a single column vector)
        %   u: Control (u_0 to u_N-1 in a single column vector) 
        %   P: Struct of parameters
        %
        % Outputs:
        %   dc_du: Partial of cost wrt the input

            % Extract variables
            [x_mat, u_mat] = obj.getInputStateMatrices(x, u);

            % Loop through and calculate the instantaneous cost
            c = 0;
            for j = 1:obj.N % Note that j = k+1
                % Extract the state and input
                uk = u_mat(:,j);
                xk = x_mat(:,j);
                xd_j = obj.xd(:,j);
                xe = xk - xd_j;

                % Calculate cost
                c = c + uk'*obj.R*uk;
                c = c + xe'*obj.Q*xe;
            end

            % Calculate terminal cost
            xe = x_mat(:,end) - obj.xd(:,end);
            c = c + xe'*obj.S*xe; % Quadratic in the error
        end

        function dL_dx_k = instantaneousPartialWrtState(obj, xk, uk, k)
        %instantaneousPartialWrtState returns dL/dxk, the partial of the
        %instantanous cost wrt the kth state
            xd_k = obj.xd(:,k);
            xe = xk - xd_k;
            dL_dx_k = 2.*xe'*obj.Q;
        end
        
        function dL_du_k = instantaneousPartialWrtControl(obj, xk, uk, k)
        %instantaneousPartialWrtControl returns dL/duk, the partial of the
        %instantaneous cost wrt the kth control
            dL_du_k = 2.*uk'*obj.R;
        end
        
        function dphi_dx = terminalPartialWrtState(obj, x_N)
        %terminalPartialWrtState returns dphi/dx, the partial of the 
        %terminal cost wrt the state    
            xd_e = obj.xd(:,end);
            xe = x_N - xd_e;
            dphi_dx = 2.*xe'*obj.S;
        end
    end
    
    %%% Desired state functions %%%
    methods
        function xd_mat = calculateDesiredState(obj, k_start)
        %obj.calculateDesiredState Calculates the desired states over the entire time
        %horizon from k = 0 to k = N
        %
        % Inputs:
        %   P: Struct of parameters
        %
        % Outputs:
        %   xd_mat: desired state where each column corresponds to a discrete index
        
            % Initialize the desired state matrix
            xd_mat = zeros(obj.n_x, obj.N+1);
            
            % Loop through all discrete time values
            col_ind = 1;
            k_ind = (0:obj.N) + k_start;
            for k = k_ind
                xd_mat(:,col_ind) = obj.getDesiredState(k);
                
                % Increment column index
                col_ind = col_ind + 1;
            end
        end

        function xd = getDesiredState(obj, k)
        %obj.getDesiredState calculates the desired state given the discrete index, k
        %
        % Inputs:
        %   k: discrete index between 0 and P.N
        %   P: Struct of parameters
        %  
        % Outputs:
        %   xd: Desired state for x_k

            % Calculate the time for which k corresponds
            t = k*obj.dt;

%             % Calcualte the state (right now it is a sinusoid)
%             xd = zeros(obj.n_x, 1);
%             xd(1) = sin(t); % Position
%             xd(2) = t; 
%             xd(3) = cos(t); % Velocity
%             xd(4) = 1;
%             xd(5) = -sin(t); % Acceleration
%             xd(6) = 0;
% %             xd(7) = -cos(t); % Jerk
% %             xd(8) = 0;
             % Calcualte the state (right now it is a sinusoid)
            xd = zeros(obj.n_x, 1);
            xd(1) = 10; % Position
            xd(2) = 10; 
            xd(3) = 0; % Velocity
            xd(4) = 0;
            xd(5) = 0; % Acceleration
            xd(6) = 0;
        end

        function ud_mat = calculateDesiredInput(obj, k_start)
        %obj.calculateDesiredInput Calculates the desired input over the entire time
        %horizon from k = 0 to k = N-1
        %
        % Inputs:
        %   P: Struct of parameters
        %
        % Outputs:
        %   ud_mat: desired input where each column corresponds to a discrete index

            % Initialize the desired state matrix
            ud_mat = zeros(obj.n_u, obj.N);

            % Loop through all discrete time values
            col_ind = 1;
            k_ind = (0:obj.N-1) + k_start;
            for k = k_ind
                ud_mat(:,col_ind) = obj.getDesiredInput(k);

                % Increment column index
                col_ind = col_ind + 1;
            end
        end

        function ud = getDesiredInput(obj, k)
        %obj.getDesiredInput calculates the desired input given the discrete index, k
        %
        % Inputs:
        %   k: discrete index between 0 and P.N
        %   P: Struct of parameters
        %  
        % Outputs:
        %   xd: Desired state for x_k

            % Calculate the time for which k corresponds
            t = k*obj.dt;

            % Calcualte the state (right now it is a sinusoid)
            ud = zeros(obj.n_u, 1);
            ud(1) = -cos(t); % Jerk
            %ud(1) = sin(t); % Snap
            ud(2) = 0;     
        end
    end
    
    %%% Plotting functions %%%
    methods
        function plotStateAndInput(obj, x, u)
            % Get the state and input in matrix form
            if size(x, 1) == obj.n_x
                x_mat = x;
                u_mat = u;
            else
                [x_mat, u_mat] = obj.getInputStateMatrices(x, u);
            end
            
            % Get time matrix
            N_plot = size(x_mat, 2);
            t_vec = linspace(0, N_plot*obj.dt, N_plot);
            
            % Create the desired state
            xd_result = zeros(obj.n_x, N_plot);
            for k = 0:(N_plot-1)
                xd_result(:,k+1) = obj.getDesiredState(k);
            end

            % Plot the state
            figure;
            names = {'x_1', 'x_2', 'xd_1', 'xd_2', 'xdd_1', 'xdd_2', 'xddd_1', 'xddd_2'};
            sub_plot_ind = 1;
            for k = 1:obj.n_x
                % Plot the desired state
                subplot(obj.n_x, 2, sub_plot_ind);
                plot(t_vec, xd_result(k,:), 'r:', 'linewidth', 3); hold on;

                % Plot the actual state
                plot(t_vec, x_mat(k,:), 'b', 'linewidth', 2);
                ylabel(names{k});
                xlabel('time (s)');

                % Update subplot index (increment by two to avoid the right column)
                sub_plot_ind = sub_plot_ind + 2;
            end
            
            % Plot the inputs
            names = {'u_1', 'u_2'};
            sub_plot_ind = 2;
            for k = 1:obj.n_u
%                 % Plot the desired state
                subplot(obj.n_x, 2, sub_plot_ind);
%                 plot(t_vec, obj.ud(k,:), 'r:', 'linewidth', 3); hold on;

                % Plot the actual state
                plot(t_vec, u_mat(k,:), 'b', 'linewidth', 2);
                ylabel(names{k});
                xlabel('time (s)');

                % Update subplot index (increment by two to avoid the left column)
                sub_plot_ind = sub_plot_ind + 2;
            end
        end

        function [h_d, h_x] = plot2dPosition(obj, x, ax, h_d, h_x)
            % Extract data
            x_mat = obj.getStateMatrix(x);

            % Plot the results
            if isempty(h_d)
                h_d = plot(ax, obj.xd(1, :), obj.xd(2, :), 'r:', 'linewidth', 3); hold on;
                h_x = plot(ax, x_mat(1,:), x_mat(2,:), 'b', 'linewidth', 2);   
            else
                set(h_d, 'xdata', obj.xd(1, :), 'ydata', obj.xd(2, :));
                set(h_x, 'xdata', x_mat(1,:), 'ydata', x_mat(2,:));
            end
        end
    end
end