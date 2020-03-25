classdef LOU
    methods(Static)
        function x = discreteSim(u, P)
        % LOU.discreteSim calculates the state given the control inputs
        %
        % Inputs:
        %   u: Control inputs (u_0 to u_N-1 in a single column vector)
        %   P: Struct of parameters
        %
        % Outputs:
        %   x: Calculated state (x_0 to x_N in a single column vector)

            % Change the inputs to a matrix form
            u_mat = LOU.getInputMatrix(u, P);

            % Initialize the state matrix
            x_mat = zeros(P.n_x, P.N+1);
            x_mat(:,1) = P.x0;

            % Loop through and update the states
            for j = 1:P.N % Note that j = k+1
                % Calculate the update for the state
                x_mat(:,j+1) = P.Abar*x_mat(:,j) + P.Bbar*u_mat(:,j);
            end

            % Convert the state matrix back to a column vector
            x = reshape(x_mat, P.n_state, 1);
        end

        %%%% Optimization functions
        function [x, u] = simultaneousOptimization(x0, u0, P)
        % LOU.simultaneousOptimization simultaneously optimizes over both the state and
        % the control, using an equality constraint to relate the two
        %
        % Inputs:
        %   x0: Initial guess for the states (x_0 to x_N in a single column vector)
        %   u0: Initial guess for the control (u_0 to u_N-1 in a single column vector)
        %   P: Struct of parameters
        %
        % Outputs:
        %   x: Optimized state (x_0 to x_N in a single column vector)
        %   u: Optimized control (u_0 to u_N-1 in a single column vector)

            % Create the aggregate state and calculate the initial cost
            y0 = [x0; u0];
            c_init = LOU.costCombined(y0, P) % Display the initial cost

            % Initialize the solver variables
            options = optimoptions(@fmincon, 'Algorithm', 'sqp'); % choose sequential-quadratic-programming
            %options = optimoptions(@fmincon, 'Algorithm', 'interior-point'); % choose an the interior-point algorithm
            options = optimoptions(options, 'SpecifyObjectiveGradient', true); % Indicate whether to use gradient
            %options = optimoptions(options, 'OptimalityTolerance', 0.1); % Tolerance for optimization
            %options.Display = 'iter'; % Have Matlab display the optimization information with each iteration

            % Define the linear inequality constraints (empty matrices because we
            % do not have any)
            A = [];
            B = [];

            % Define the linear equality constraints
            [Aeq, Beq] = LOU.calculateStepwiseEquality(P);
            %[Aeq, Beq] = LOU.calculateFullEffectEquality(P);

            % Define the upper and lower bounds (empty matrices because we do not
            % have any)
            lb = []; % No upper or lower bounds
            ub = [];

            % Matlab call:
            [y, final_cost] = fmincon(@(y) LOU.costCombined(y, P), y0, A, B, Aeq, Beq, lb, ub, [], options);
            disp(['Final cost = ' num2str(final_cost)]);

            % Extract the state and control from the variable y
            [x, u] = LOU.extractStateAndControl(y, P);    
        end

        function [x, u] = sequentialOptimization(u0, P)
        % LOU.sequentialOptimization optimizes over the control. At each iteration the
        % control is used to calculate the state which are both used to calculate
        % the cost
        %
        % Inputs:
        %   u0: Initial guess for the control (u_0 to u_N-1 in a single column vector)
        %   P: Struct of parameters
        %
        % Outputs:
        %   x: Optimized state (x_1 to x_N in a single column vector)
        %   u: Optimized control (u_0 to u_N-1 in a single column vector)

            % Output the initial cost
            c_init = LOU.costSequential(u0, P)

            % Initialize the solve variables
            options = optimoptions(@fmincon, 'Algorithm', 'sqp'); % choose sequential-quadratic-programming
            %options = optimoptions(@fmincon, 'Algorithm', 'interior-point'); % choose an the interior-point algorithm
            options = optimoptions(options, 'SpecifyObjectiveGradient', true); % Indicate whether to use gradients (Note that there are no constraint gradients)
            options = optimoptions(options, 'OptimalityTolerance', 0.1); % Tolerance for optimization
            %options.Display = 'iter'; % Have Matlab display the optimization information with each iteration

            % Define the linear inequality constraints (empty matrices because we
            % do not have any)
            A = [];
            B = [];

            % Define the linear equality constraints (empty matrices because we do
            % not have any)
            Aeq = []; % No equality constraints
            Beq = [];

            % Define the upper and lower bounds (empty matrices because we do not
            % have any)
            lb = []; % No upper or lower bounds
            ub = []; 

            % Matlab call:
            [u, final_cost] = fmincon(@(u_in) LOU.costSequential(u_in, P), u0, A, B, Aeq, Beq, lb, ub, [], options);
            disp(['Final cost = ' num2str(final_cost)]);

            % Simulate the state forward in time to be able to output the result
            x = LOU.discreteSim(u, P);    
        end

        function [A_eq, B_eq] = calculateStepwiseEquality(P)
        %LOU.calculateStepwiseEquality Calculates the equality constraint one step at a
        %time
        %
        % Inputs:
        %   P: Struct of parameters
        % 
        % Outputs: 
        %   Matrices such that A_eq*y = B_eq

            %% Calculate the state portion of the constraint
            A_eq_state = -eye(P.n_state); % Portion corresponding to next state

            % Lower diagonal term is a block matrix of Abar matrices
            ind_row = (P.n_x+1):(2*P.n_x); % Row index for the matrix
            ind_col = 1:P.n_x; % Column index for the matrix
            for k = 1:P.N
                % Add the state update matrix
                A_eq_state(ind_row, ind_col) = P.Abar;

                % Increment the indices
                ind_row = ind_row + P.n_x;
                ind_col = ind_col + P.n_x;
            end

            %% Calculate the input portion of the constraint
            A_eq_ctrl = zeros(P.n_state, P.n_ctrl);
            ind_row = (P.n_x+1):(2*P.n_x); % Row index for the matrix
            ind_col = 1:P.n_u; % Column index for the matrix
            for k = 1:P.N
                A_eq_ctrl(ind_row, ind_col) = P.Bbar;

                % Update indices
                ind_row = ind_row + P.n_x;
                ind_col = ind_col + P.n_u;
            end

            % Put the state and input portions of the matrix together
            A_eq = [A_eq_state, A_eq_ctrl];

            %% Calculate the B_eq matrix
            B_eq = zeros(P.n_state, 1);
            B_eq(1:P.n_x) = -P.x0; % The first state must be equal to the initialization


        %     %% Test the gradient (should be commented out)
        %     % Calculate the numerical jacobian
        %     A_eq_num = jacobianest(@(y_in) LOU.stepWiseEqualityConstraint(y_in, P), rand(P.n_state+P.n_ctrl, 1)*100);
        %     
        %     % Calculate the error
        %     A_eq_err = norm(A_eq - A_eq_num, 'fro')
        end

        function h = stepWiseEqualityConstraint(y, P)
            % Extract state and input
            [x, u] = LOU.extractStateAndControl(y, P);

            % Calculate the state matrices
            [x_mat, u_mat] = LOU.getInputStateMatrices(x, u, P);

            % Calculate the constraint
            h = zeros(P.n_state, 1);
            h(1:P.n_x) = -x_mat(:,1) + P.x0;
            ind_h = (P.n_x+1):(2*P.n_x);
            for j = 1:P.N % Note that j = k+1
                % Calculate the constraint
                h(ind_h) = P.Abar*x_mat(:,j)+P.Bbar*u_mat(:,j) - x_mat(:,j+1);

                % Increment the index
                ind_h = ind_h + P.n_x;
            end
        end

        function [A_eq, B_eq] = calculateFullEffectEquality(P)
        %LOU.calculateFullEffectEquality Calculates the equality constraint using steps
        %from the initial state and inputs to the state in question
        %
        % Inputs:
        %   P: Struct of parameters
        % 
        % Outputs: 
        %   Matrices such that A_eq*y = B_eq

            %% Calculate the state portion of the constraint
            A_eq_state = -eye(P.n_state); % Portion corresponding to next state

            % Lower diagonal term is a block matrix of Abar matrices
            ind_row = (P.n_x+1):(2*P.n_x); % Row index for the matrix
            ind_col = 1:P.n_x; % Column index for the matrix
            for k = 1:P.N
                % Add the state update matrix
                A_eq_state(ind_row, ind_col) = P.Abar^k;

                % Increment the indices (note, the column matrix does not update)
                ind_row = ind_row + P.n_x;        
            end

            %% Calculate the input portion of the constraint
            A_eq_ctrl = zeros(P.n_state, P.n_ctrl);
            ind_row = (P.n_x+1):(2*P.n_x); % Row index for the matrix
            for blk_row = 1:P.N
                % loop through columns
                ind_col = 1:P.n_u;
                for col = (blk_row-1):-1:0
                    A_eq_ctrl(ind_row, ind_col) = P.Abar^col*P.Bbar;

                    % Update column indices
                    ind_col = ind_col + P.n_u;
                end

                % Update row indices
                ind_row = ind_row + P.n_x;
            end

            % Put the state and input portions of the matrix together
            A_eq = [A_eq_state, A_eq_ctrl];

            %% Calculate the B_eq matrix
            B_eq = zeros(P.n_state, 1);
            B_eq(1:P.n_x) = -P.x0; % The first state must be equal to the initialization

        %     %% Test the full constraint
        %     u = rand(P.n_ctrl, 1)*100;
        %     x = LOU.discreteSim(u, P);
        %     h = LOU.stepWiseFullEffectConstraint([x; u], P)
        %     
        %     %% Test the gradient (should be commented out)
        %     % Calculate the numerical jacobian
        %     A_eq_num = jacobianest(@(y_in) LOU.stepWiseFullEffectConstraint(y_in, P), rand(P.n_state+P.n_ctrl, 1)*100);
        %     
        %     % Calculate the error
        %     A_eq_err = norm(A_eq - A_eq_num, 'fro')
        end

        function h = stepWiseFullEffectConstraint(y, P)
            % Extract state and input
            [x, u] = LOU.extractStateAndControl(y, P);

            % Calculate the state matrices
            [x_mat, u_mat] = LOU.getInputStateMatrices(x, u, P);

            % Calculate the constraint
            h = zeros(P.n_state, 1);
            h(1:P.n_x) = -x_mat(:,1) + P.x0;
            ind_row = (P.n_x+1):(2*P.n_x);
            for blk_row = 1:P.N % Note that blk_row = k+1
                % Calculate the effect on the combined state of the initial state
                x_comb = P.Abar^blk_row*x_mat(:,1);

                % Calculate the effect on the input
                ind_u = 1;
                for exp = blk_row:-1:1
                    x_comb = x_comb + P.Abar^(exp-1)*P.Bbar*u_mat(:,ind_u);
                    ind_u = ind_u + 1;
                end

                % Calculate the constraint
                h(ind_row) = x_comb - x_mat(:,blk_row+1);

                % Increment the index
                ind_row = ind_row + P.n_x;
            end
        end
        %%%% Cost functions
        function [c, dc_dy] = costCombined(y, P)
        %LOU.costCombined calculates the cost and gradient of the cost with respect to
        %the aggregate state, y = [x; u]
        %
        % Inputs:
        %   y: Column vector with both states and input (y = [x; u])
        %   P: Struct of parameters
        %
        % Outputs:
        %   c: cost
        %   dc_dy: partial derivative of the cost wrt y

            % Separate out the states
            [x, u] = LOU.extractStateAndControl(y, P);

            % Calculate the cost
            c = LOU.cost(x, u, P);

            % Calculate the partial
            if nargout > 1 % Only calculate the partial if the calling function wants it
                dc_dy = [LOU.calculatePartialWrtState(x,u,P), ... % dc/dx
                         LOU.calculatePartialWrtInput(x,u,P)];    % dc/du
            else
                dc_dy = []; % Output an empty value if not needed
            end

        %     %% Check partials (Comment this code when working)
        %     % Calculate individual partials
        %     dc_dx = LOU.calculatePartialWrtState(x,u,P);
        %     dc_du = LOU.calculatePartialWrtInput(x,u,P);
        %     
        %     % Calculate numerical partials
        %     dc_dx_num = jacobianest(@(x_in) LOU.cost(x_in, u, P), x);
        %     dc_du_num = jacobianest(@(u_in) LOU.cost(x, u_in, P), u);
        %     
        %     % Calculate error
        %     dc_dx_err = norm(dc_dx - dc_dx_num, 'fro')
        %     dc_du_err = norm(dc_du - dc_du_num, 'fro')

        end

        function [c, dc_du] = costSequential(u, P)
        %LOU.costSequential calculates the cost and gradient of the cost with respect to
        %the input, u
        %
        % Inputs:
        %   u: Input control (u_0 to u_N-1 in a single column vector)
        %   P: Struct of parameters
        %
        % Outputs:
        %   c: cost
        %   dc_du: partial derivative of the cost wrt u

            % Calculate the state based upon the controls
            x = LOU.discreteSim(u, P);

            % Calcualte the cost
            c = LOU.cost(x, u, P);

            % Calculate the partial
            if nargout > 1
                dc_du = LOU.sequentialPartial(x, u, P);        
            else
                dc_du = [];
            end
        end

        function c = cost(x, u, P)
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
            [x_mat, u_mat] = LOU.getInputStateMatrices(x, u, P);

            % Loop through and calculate the instantaneous cost
            c = 0;
            for j = 1:P.N % Note that j = k+1
                % Extract the state and input
                uk = u_mat(:,j);
                xk = x_mat(:,j);
                xd = P.xd(:,j);
                xe = xk - xd;

                % Calculate cost
                c = c + uk'*P.R*uk;
                c = c + xe'*P.Q*xe;
            end

            % Calculate terminal cost
            xe = x_mat(:,end) - P.xd(:,end);
            c = c + xe'*P.S*xe; % Quadratic in the error
        end

        function dc_dx = calculatePartialWrtState(x,u,P)
        %LOU.calculatePartialWrtState Calculates the partial of the cost wrt each state
        %
        % Inputs:
        %   x: State (x_0 to x_N in a single column vector)
        %   u: Control (u_0 to u_N-1 in a single column vector) 
        %   P: Struct of parameters
        %
        % Outputs:
        %   dc_dx: partial of the cost wrt the state

            % Reformulate the states to be in terms of the column vectors
            [x_mat, ~] = LOU.getInputStateMatrices(x, u, P);

            % Initialize the gradient
            ind_x = 1:P.n_x;
            dc_dx = zeros(1, P.n_state);
            for j = 1:P.N % Note that j = k+1
                % Get the state error
                xk = x_mat(:,j);
                xd = P.xd(:,j);
                xe = xk - xd;

                % Calculate the partial (assuming Q = Q')
                dc_dx(ind_x) = 2.*xe'*P.Q;

                % Increment the partial index
                ind_x = ind_x + P.n_x;
            end

            % Calculate the partial for the terminal state
            xk = x_mat(:,end);
            xd = P.xd(:,end);
            xe = xk - xd;
            dc_dx(ind_x) = 2.*xe'*P.S;    
        end

        function dc_du = calculatePartialWrtInput(x,u,P)
        %LOU.calculatePartialWrtState Calculates the partial of the cost wrt each input
        %
        % Inputs:
        %   x: State (x_0 to x_N in a single column vector)
        %   u: Control (u_0 to u_N-1 in a single column vector) 
        %   P: Struct of parameters
        %
        % Outputs:
        %   dc_du: partial of the cost wrt the input

            % Reformulate the states to be in terms of the column vectors
            [~, u_mat] = LOU.getInputStateMatrices(x, u, P);

            % Initialize the gradient
            ind_u = 1:P.n_u;
            dc_du = zeros(1, P.n_ctrl);
            for j = 1:P.N % Note that j = k+1
                % Get the control input
                uk = u_mat(:,j);

                % Calculate the partial (assuming R = R')
                dc_du(ind_u) = 2.*uk'*P.R;

                % Increment the partial index
                ind_u = ind_u + P.n_u;
            end
        end

        %%% Sequential optimization functions
        function dc_du = sequentialPartial(x, u, P)
        %LOU.sequentialPartial: Calculates the partial of the cost wrt the input u
        % Note that the partial takes the form:
        %       dc/duk = dL/duk + lam_{k+1}^T df/duk
        % Inputs: 
        %   x: State (x_1 to x_N in a single column vector)
        %   u: Control (u_0 to u_N-1 in a single column vector) 
        %   P: Struct of parameters
        %
        % Outputs:
        %   dc_du: Partial of cost wrt the input

            % Initialize the partial
            dc_du = zeros(P.n_u, P.N); % We will use one column per gradient and then reshape to make a row vector at the end

            % Reshape vectors for easy access
            [x_mat, u_mat] = LOU.getInputStateMatrices(x, u, P);

            % Initialize final lagrange multiplier (lam_N = dphi/dx(x_N)
            ind_x = size(x_mat, 2); % Final column index corresponds to the final state
            lam_kp1 = (2.*(x_mat(:,ind_x)-P.xd(:, ind_x))'*P.S)'; % dphi/dx(x_N) <-- This variable is used as lam_{k+1}
            lam_k = lam_kp1; % duplicate for initializaiton purposes (the loop moves backward 
                             % in time so at the beginning it changes iterations by
                             % setting lam_{k+1} = lam_k as k has been decremented

            % Simulate backward in time
            ind_u = size(u_mat, 2); % Index of the uk at iteration k (start with last column for k = P.N-1
            ind_x = ind_x - 1; % Index of xk at iteration k (start with second to last column for k = P.N-1)
            for k = (P.N-1):-1:0 % Simulate backwards in time
                % Extract the state and input
                uk = u_mat(:,ind_u); % Input at iteration k
                xk = x_mat(:, ind_x); % State at iteration k
                lam_kp1 = lam_k; % Update k index for the lagrange multipliers - \lambda at iteration k+1

                % Calculate partials needed for updates
                dLk_duk = 2.*uk'*P.R;
                dfk_duk = P.Bbar;
                dfk_dxk = P.Abar;

                % Calculate partial of cost wrt state
                xd = P.xd(:,ind_x);
                xe = xk - xd;
                dLk_dxk = 2.*xe'*P.Q; % Instantaneous cost does not depend on xk

                % Calculate partial
                dc_du(:, ind_u) = (dLk_duk + lam_kp1'*dfk_duk)'; % Transposed to fit in temporary variable

                % Update Lagrange muliplier (moving backward in time)
                lam_k = dLk_dxk' + dfk_dxk'*lam_kp1;

                % Update the indices for the next iteration
                ind_u = ind_u - 1;
                ind_x = ind_x - 1;
            end

            % Reshape partial to be proper output
            dc_du = reshape(dc_du, 1, P.n_ctrl); % changes it from partial in columns for each iteration to one single row
        end


        %%%% Desired state functions
        function xd_mat = calculateDesiredState(P, k_start)
        %LOU.calculateDesiredState Calculates the desired states over the entire time
        %horizon from k = 0 to k = N
        %
        % Inputs:
        %   P: Struct of parameters
        %
        % Outputs:
        %   xd_mat: desired state where each column corresponds to a discrete index
        
            % Initialize the desired state matrix
            xd_mat = zeros(P.n_x, P.N+1);
            
            % Loop through all discrete time values
            col_ind = 1;
            k_ind = [0:P.N] + k_start;
            for k = k_ind
                xd_mat(:,col_ind) = LOU.getDesiredState(k, P);
                
                % Increment column index
                col_ind = col_ind + 1;
            end
        end

%         function xd_mat = calculateDesiredState(P)
%         %LOU.calculateDesiredState Calculates the desired states over the entire time
%         %horizon from k = 0 to k = N
%         %
%         % Inputs:
%         %   P: Struct of parameters
%         %
%         % Outputs:
%         %   xd_mat: desired state where each column corresponds to a discrete index
% 
%             % Initialize the desired state matrix
%             xd_mat = zeros(P.n_x, P.N+1);
% 
%             waypoints = [[-5; -5],[-5; 5],[5; 5],[5; -5]];
%             ds = 0.5;
% 
%             p1 = waypoints(:,1);
%             p2 = waypoints(:,2);
%             n = 2;
%             q = p1;
%             dq = (p2-p1)/norm(p2-p1) .* ds;
% 
%             for k = 1:(P.N+1)
%                 xd_mat(1:2,k) = q;
% 
%                 % Update q
%                 q = q + dq;
% 
%                 % Update points
%                 if norm(q-p2) < ds
%                     p1 = p2;
%                     n = n+1;
%                     if n > size(waypoints, 2)
%                         n = 1;
%                     end
%                     p2 = waypoints(:,n);
%                     dq = (p2-q)/norm(p2-q) .* ds;
%                 end
%             end
% 
%         end

        function xd = getDesiredState(k, P)
        %LOU.getDesiredState calculates the desired state given the discrete index, k
        %
        % Inputs:
        %   k: discrete index between 0 and P.N
        %   P: Struct of parameters
        %  
        % Outputs:
        %   xd: Desired state for x_k

            % Calculate the time for which k corresponds
            t = k*P.dt;

            % Calcualte the state (right now it is a sinusoid)
            xd = zeros(P.n_x, 1);
            xd(1) = sin(t); % Position
            xd(2) = t; 
            xd(3) = cos(t); % Velocity
            xd(4) = 1;
            xd(5) = -sin(t); % Acceleration
            xd(6) = 0;
%             xd(7) = -cos(t); % Jerk
%             xd(8) = 0;
        end

        function ud_mat = calculateDesiredInput(P, k_start)
        %LOU.calculateDesiredInput Calculates the desired input over the entire time
        %horizon from k = 0 to k = N-1
        %
        % Inputs:
        %   P: Struct of parameters
        %
        % Outputs:
        %   ud_mat: desired input where each column corresponds to a discrete index

            % Initialize the desired state matrix
            ud_mat = zeros(P.n_u, P.N);

            % Loop through all discrete time values
            col_ind = 1;
            k_ind = [0:P.N-1] + k_start;
            for k = k_ind
                ud_mat(:,col_ind) = LOU.getDesiredInput(k, P);

                % Increment column index
                col_ind = col_ind + 1;
            end
        end

        function ud = getDesiredInput(k, P)
        %LOU.getDesiredInput calculates the desired input given the discrete index, k
        %
        % Inputs:
        %   k: discrete index between 0 and P.N
        %   P: Struct of parameters
        %  
        % Outputs:
        %   xd: Desired state for x_k

            % Calculate the time for which k corresponds
            t = k*P.dt;

            % Calcualte the state (right now it is a sinusoid)
            ud = zeros(P.n_u, 1);
            ud(1) = -cos(t); % Jerk
            %ud(1) = sin(t); % Snap
            ud(2) = 0;     
        end

        %%%% State access functions
        function [x_mat, u_mat] = getInputStateMatrices(x, u, P)
        % LOU.getInputStateMatrices converts the vector state and input to a matrix where each column
        % corresponds to an iteration time
        %
        % Inputs:
        %   x: State (x_0 to x_N in a single column vector)
        %   u: Control inputs (u_0 to u_N-1 in a single column vector)
        %   P: Struct of parameters
        %  
        % Outputs:
        %   x_mat: Matrix of states, number of rows = P.n_x and number of columns =
        %          P.N+1
        %   u_mat: Matrix of inputs, number of rows = P.n_u and number of columns =
        %          P.N
            x_mat = LOU.getStateMatrix(x, P);
            u_mat = LOU.getInputMatrix(u, P);
        end

        function u_mat = getInputMatrix(u, P)
        % LOU.getInputMatrix converts the vector input to a matrix where each column
        % corresponds to an iteration time
        %
        % Inputs:
        %   u: Control inputs (u_0 to u_N-1 in a single column vector)
        %   P: Struct of parameters
        %  
        % Outputs:
        %   u_mat: Matrix of inputs, number of rows = P.n_u and number of columns =
        %   P.N
            u_mat = reshape(u, P.n_u, P.N);
        end

        function x_mat = getStateMatrix(x, P)
        % LOU.getStateMatrix converts the vector state to a matrix where each column
        % corresponds to an iteration time
        %
        % Inputs:
        %   x: State (x_0 to x_N in a single column vector)
        %   P: Struct of parameters
        %  
        % Outputs:
        %   x_mat: Matrix of states, number of rows = P.n_x and number of columns =
        %   P.N+1
            x_mat = reshape(x, P.n_x, P.N+1);
        end

        function [x, u] = extractStateAndControl(y, P)
        % LOU.sequentialOptimization extracts the state, x, and control, u, from the
        % aggregate optimization variables y = [x; u]
        %
        % Inputs:
        %   y: Column vector with both states and input (y = [x; u])
        %   P: Struct of parameters
        %
        % Outputs:
        %   x: Extracted state (x_0 to x_N in a single column vector)
        %   u: Extracted control (u_0 to u_N-1 in a single column vector) 
        %

            % Separate out the states
            x = y(1:P.n_state);
            u = y(P.n_state+1:end);
        end

        function [Abar, Bbar] = calculateDiscreteTimeMatrices(P)

            % Calculate the discrete time state matrix
            Abar = expm(P.A*P.dt);

            % Calculate the discrete time input matrix
            b0 = zeros(P.n_x*P.n_x, 1);
            [~, Bbar_mat] = ode45(@(t, x) bBarDynamics(t, x, P), [0, P.dt], b0);
            Bbar_mat = Bbar_mat';
            Bbar = reshape(Bbar_mat(:, end), P.n_x, P.n_x)*P.B;

            % Dynamics function for calculating Bbar
            function b_dot = bBarDynamics(tau, b_bar, P)
                b_dot = expm(P.A*(P.dt-tau)); % Calculate the time derivative at tau
                b_dot = reshape(b_dot, P.n_x*P.n_x, 1); % Reshape to a column vector
            end
        end

        %%%% Test and plotting functions
        function plotStateAndInput(x, u, P)
            % Get the state and input in matrix form
            [x_mat, u_mat] = LOU.getInputStateMatrices(x, u, P);

            % Plot the state
            figure;
            names = {'x_1', 'x_2', 'xd_1', 'xd_2', 'xdd_1', 'xdd_2', 'xddd_1', 'xddd_2'};
            sub_plot_ind = 1;
            for k = 1:P.n_x
                % Plot the desired state
                subplot(P.n_x, 2, sub_plot_ind);
                plot(P.xd(k,:), 'r:', 'linewidth', 3); hold on;

                % Plot the actual state
                plot(x_mat(k,:), 'b', 'linewidth', 2);
                ylabel(names{k});

                % Update subplot index (increment by two to avoid the right column)
                sub_plot_ind = sub_plot_ind + 2;
            end

            % Plot the inputs
            names = {'u_1', 'u_2'};
            sub_plot_ind = 2;
            for k = 1:P.n_u
                % Plot the desired state
                subplot(P.n_x, 2, sub_plot_ind);
                plot(P.ud(k,:), 'r:', 'linewidth', 3); hold on;

                % Plot the actual state
                plot(u_mat(k,:), 'b', 'linewidth', 2);
                ylabel(names{k});

                % Update subplot index (increment by two to avoid the left column)
                sub_plot_ind = sub_plot_ind + 2;
            end
        end

        function [h_d, h_x] = plot2dPosition(x, P, ax, h_d, h_x)
            % Extract data
            x_mat = LOU.getStateMatrix(x, P);

            % Plot the results
            if isempty(h_d)
                h_d = plot(ax, P.xd(1, :), P.xd(2, :), 'r:', 'linewidth', 3); hold on;
                h_x = plot(ax, x_mat(1,:), x_mat(2,:), 'b', 'linewidth', 2);   
            else
                set(h_d, 'xdata', P.xd(1, :), 'ydata', P.xd(2, :));
                set(h_x, 'xdata', x_mat(1,:), 'ydata', x_mat(2,:));
            end
        end

        function testDiscreteDynamics(P)
            % Choose a random first state and control input
            x0 = rand(P.n_x, 1)*100;
            u = rand(P.n_u, 1)*10;

            % Take a continuous-time step for the sampling interval
            [tmat, xmat] = ode45(@(t, x) LOU.continuousDynamics(t, x, u, P), [0 P.dt], x0);
            xmat = xmat';

            % Take a discrete-time step for the sampling interval
            x1_disc = P.Abar*x0 + P.Bbar*u;

            % Plot the continuous and discrete trajectories
            figure;
            names = {'x_1', 'x_2', 'xd_1', 'xd_2', 'xdd_1', 'xdd_2', 'xddd_1', 'xddd_2'};
            for k = 1:P.n_x
                % Plot the continuous trajectory
                subplot(P.n_x, 1, k);
                plot(tmat, xmat(k,:), 'b', 'linewidth', 2); hold on;

                % Plot the discrete trajectory
                plot([0 P.dt], [x0(k) x1_disc(k)], 'ro', 'linewidth', 2);
                ylabel(names{k});        
            end
            xlabel('time');
        end

        function xdot = continuousDynamics(t, x, u, P)
        xdot = P.A * x + P.B*u;
        end

    end
end