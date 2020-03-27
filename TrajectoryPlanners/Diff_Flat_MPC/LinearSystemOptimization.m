classdef LinearSystemOptimization
    properties
        A % continuous time state matrix
        B % continuous time input matrix
        Abar % discrete time state matrix
        Bbar % discrete time state matrix
        dt = 0.1; % Discrete time interval
        n_x % Number of states
        n_u % Number of inputs
        
        % Optimization variables
        N % Number of steps
        n_ctrl % Control from u_0 to u_{N-1}
        n_state % State from x_0 to x_N        
        
        % Optimization parameters
        P_simult % Parameters for simultaneous optimization
        P_seq % Parameters for sequential optimization
    end
    
    properties(SetAccess=protected)
        x0 % Initial state
    end
    
    %%% Problem specific functions %%%
    methods(Abstract)
        c = cost(obj, x, u); %cost calculates the cost given the state and inputs
        dL_dx_k = instantaneousPartialWrtState(obj, xk, uk, k); %instantaneousPartialWrtState returns dL/dxk, the partial of the
                                                                %instantanous cost wrt the kth state
        dL_du_k = instantaneousPartialWrtControl(obj, xk, uk, k); %instantaneousPartialWrtControl returns dL/duk, the partial of the
                                                                  %instantaneous cost wrt the kth control
        dphi_dx = terminalPartialWrtState(obj, x_N); %terminalPartialWrtState returns dphi/dx, the partial of the 
                                                     %terminal cost wrt the state                
    end
    
    %%% Initialization functions %%%
    methods
        function obj = LinearSystemOptimization(A, B, N)
            %LOU Construct an instance of this class
            %
            % Inputs:
            %   A: Continous-time state matrix
            %   B: Continuous-time input matrix
            
            % Store inputs
            obj.A = A;
            obj.B = B;
            obj.N = N;
            
            % Process state variables
            obj.n_x = size(A, 1);
            obj.n_u = size(B, 2);
            
            obj = obj.initializeParameters();
        end
        
        function obj = initializeParameters(obj)
        %initializeParameters Initializes the parameter optimization
        %parameters
            % Calculate discrete dynamics
            [obj.Abar, obj.Bbar] = obj.calculateDiscreteTimeMatrices();
            
            % Initialize optimization variables
            obj.n_ctrl = obj.n_u*obj.N; % Control from u_0 to u_{N-1}
            obj.n_state = obj.n_x*(obj.N+1); % State from x_0 to x_N
            obj.x0 = zeros(obj.n_x, 1);
            
            %% Initailize simultaneous optimization parameters
            obj.P_simult.options = optimoptions(@fmincon, 'Algorithm', 'sqp'); % choose sequential-quadratic-programming
            %obj.P_simult.options = optimoptions(@fmincon, 'Algorithm', 'interior-point'); % choose an the interior-point algorithm
            obj.P_simult.options = optimoptions(obj.P_simult.options, 'SpecifyObjectiveGradient', true); % Indicate whether to use gradient
            %obj.P_simult.options = optimoptions(obj.P_simult.options, 'OptimalityTolerance', 0.1); % Tolerance for optimization
            %obj.P_simult.options.Display = 'iter'; % Have Matlab display the optimization information with each iteration
            obj.P_simult.options.Display = 'off';

            % Define the linear inequality constraints (empty matrices because we
            % do not have any)
            obj.P_simult.A_ineq = [];
            obj.P_simult.B_ineq = [];

            % Define the linear equality constraints
            [obj.P_simult.Aeq, obj.P_simult.Beq] = obj.calculateStepwiseEquality();
            %[obj.P_simult.Aeq, obj.P_simult.Beq] = obj.calculateFullEffectEquality();

            % Define the upper and lower bounds (empty matrices because we do not
            % have any)
            obj.P_simult.lb = []; % No upper or lower bounds
            obj.P_simult.ub = [];
            
            %% Initialize sequentiaion optimization parameters
            % Initialize the solve variables
            obj.P_seq.options = optimoptions(@fmincon, 'Algorithm', 'sqp'); % choose sequential-quadratic-programming
            %obj.P_seqoptions = optimoptions(@fmincon, 'Algorithm', 'interior-point'); % choose an the interior-point algorithm
            obj.P_seq.options = optimoptions(obj.P_seq.options, 'SpecifyObjectiveGradient', true); % Indicate whether to use gradients (Note that there are no constraint gradients)
            obj.P_seq.options = optimoptions(obj.P_seq.options, 'OptimalityTolerance', 0.1); % Tolerance for optimization
            %obj.P_seqoptions.Display = 'iter'; % Have Matlab display the optimization information with each iteration
            obj.P_seq.options.Display = 'off'; % Have Matlab display the optimization information with each iteration

            % Define the linear inequality constraints (empty matrices because we
            % do not have any)
            obj.P_seq.A_ineq = [];
            obj.P_seq.B_ineq = [];

            % Define the linear equality constraints (empty matrices because we do
            % not have any)
            obj.P_seq.Aeq = []; % No equality constraints
            obj.P_seq.Beq = [];

            % Define the upper and lower bounds (empty matrices because we do not
            % have any)
            obj.P_seq.lb = []; % No upper or lower bounds
            obj.P_seq.ub = []; 
        end 
        
        function obj = setInitialState(obj, x0)
        %setInitialState stores the initial state
            % Store the initial state
            obj.x0 = x0;
            
            % Set the initial state within the simultaneous optimization
            % bounds
            obj.P_simult.Beq(1:obj.n_x) = -obj.x0;
        end
    end
    
    %%% Simulation functions %%%
    methods
        function x = discreteSim(obj, u)
        % obj.discreteSim calculates the state given the control inputs
        %
        % Inputs:
        %   u: Control inputs (u_0 to u_N-1 in a single column vector)
        %   P: Struct of parameters
        %
        % Outputs:
        %   x: Calculated state (x_0 to x_N in a single column vector)

            % Change the inputs to a matrix form
            u_mat = obj.getInputMatrix(u);

            % Initialize the state matrix
            x_mat = zeros(obj.n_x, obj.N+1);
            x_mat(:,1) = obj.x0;

            % Loop through and update the states
            for j = 1:obj.N % Note that j = k+1
                % Calculate the update for the state
                x_mat(:,j+1) = obj.Abar*x_mat(:,j) + obj.Bbar*u_mat(:,j);
            end

            % Convert the state matrix back to a column vector
            x = reshape(x_mat, obj.n_state, 1);
        end
        
        function [x_mat, u_mat] = getInputStateMatrices(obj, x, u)
        % obj.getInputStateMatrices converts the vector state and input to a matrix where each column
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
            x_mat = obj.getStateMatrix(x);
            u_mat = obj.getInputMatrix(u);
        end

        function u_mat = getInputMatrix(obj, u)
        % obj.getInputMatrix converts the vector input to a matrix where each column
        % corresponds to an iteration time
        %
        % Inputs:
        %   u: Control inputs (u_0 to u_N-1 in a single column vector)
        %   P: Struct of parameters
        %  
        % Outputs:
        %   u_mat: Matrix of inputs, number of rows = P.n_u and number of columns =
        %   P.N
            u_mat = reshape(u, obj.n_u, obj.N);
        end

        function x_mat = getStateMatrix(obj, x)
        % obj.getStateMatrix converts the vector state to a matrix where each column
        % corresponds to an iteration time
        %
        % Inputs:
        %   x: State (x_0 to x_N in a single column vector)
        %   P: Struct of parameters
        %  
        % Outputs:
        %   x_mat: Matrix of states, number of rows = P.n_x and number of columns =
        %   P.N+1
            x_mat = reshape(x, obj.n_x, obj.N+1);
        end

        function [x, u] = extractStateAndControl(obj, y)
        % obj.sequentialOptimization extracts the state, x, and control, u, from the
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
            x = y(1:obj.n_state);
            u = y(obj.n_state+1:end);
        end

        function [Abar, Bbar] = calculateDiscreteTimeMatrices(obj)

            % Calculate the discrete time state matrix
            Abar = expm(obj.A*obj.dt);

            % Calculate the discrete time input matrix
            b0 = zeros(obj.n_x*obj.n_x, 1);
            [~, Bbar_mat] = ode45(@(t, x) obj.bBarDynamics(t, x), [0, obj.dt], b0);
            Bbar_mat = Bbar_mat';
            Bbar = reshape(Bbar_mat(:, end), obj.n_x, obj.n_x)*obj.B;
        end
        
        % Dynamics function for calculating Bbar
        function b_dot = bBarDynamics(obj, tau, ~)
            b_dot = expm(obj.A*(obj.dt-tau)); % Calculate the time derivative at tau
            b_dot = reshape(b_dot, obj.n_x*obj.n_x, 1); % Reshape to a column vector
        end
    end
    
    %%% Optimization functions %%%
    methods
        function [x, u] = simultaneousOptimization(obj, x0, u0)
        % obj.simultaneousOptimization simultaneously optimizes over both the state and
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
            %c_init = obj.costCombined(y0) % Display the initial cost

            

            % Matlab call:
            [y, final_cost] = fmincon(@(y) obj.costCombined(y), y0, obj.P_simult.A_ineq, ...
                obj.P_simult.B_ineq, obj.P_simult.Aeq, obj.P_simult.Beq, obj.P_simult.lb, ...
                obj.P_simult.ub, [], obj.P_simult.options);
            %disp(['Final cost = ' num2str(final_cost)]);

            % Extract the state and control from the variable y
            [x, u] = obj.extractStateAndControl(y);    
        end

        function [x, u] = sequentialOptimization(obj, u0)
        % obj.sequentialOptimization optimizes over the control. At each iteration the
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

            % Matlab call:
            [u, final_cost] = fmincon(@(u_in) obj.costSequential(u_in), u0, ...
                obj.P_seq.A_ineq, obj.P_seq.B_ineq, obj.P_seq.Aeq, obj.P_seq.Beq, ...
                obj.P_seq.lb, obj.P_seq.ub, [], obj.P_seq.options);
            %disp(['Final cost = ' num2str(final_cost)]);

            % Simulate the state forward in time to be able to output the result
            x = obj.discreteSim(u);    
        end

        function [A_eq, B_eq] = calculateStepwiseEquality(obj)
        %obj.calculateStepwiseEquality Calculates the equality constraint one step at a
        %time
        %
        % Inputs:
        %   P: Struct of parameters
        % 
        % Outputs: 
        %   Matrices such that A_eq*y = B_eq

            %% Calculate the state portion of the constraint
            A_eq_state = -eye(obj.n_state); % Portion corresponding to next state

            % Lower diagonal term is a block matrix of Abar matrices
            ind_row = (obj.n_x+1):(2*obj.n_x); % Row index for the matrix
            ind_col = 1:obj.n_x; % Column index for the matrix
            for k = 1:obj.N
                % Add the state update matrix
                A_eq_state(ind_row, ind_col) = obj.Abar;

                % Increment the indices
                ind_row = ind_row + obj.n_x;
                ind_col = ind_col + obj.n_x;
            end

            %% Calculate the input portion of the constraint
            A_eq_ctrl = zeros(obj.n_state, obj.n_ctrl);
            ind_row = (obj.n_x+1):(2*obj.n_x); % Row index for the matrix
            ind_col = 1:obj.n_u; % Column index for the matrix
            for k = 1:obj.N
                A_eq_ctrl(ind_row, ind_col) = obj.Bbar;

                % Update indices
                ind_row = ind_row + obj.n_x;
                ind_col = ind_col + obj.n_u;
            end

            % Put the state and input portions of the matrix together
            A_eq = [A_eq_state, A_eq_ctrl];

            %% Calculate the B_eq matrix
            B_eq = zeros(obj.n_state, 1);
            B_eq(1:obj.n_x) = -obj.x0; % The first state must be equal to the initialization


        %     %% Test the gradient (should be commented out)
        %     % Calculate the numerical jacobian
        %     A_eq_num = jacobianest(@(y_in) obj.stepWiseEqualityConstraint(y_in, P), rand(P.n_state+P.n_ctrl, 1)*100);
        %     
        %     % Calculate the error
        %     A_eq_err = norm(A_eq - A_eq_num, 'fro')
        end

        function h = stepWiseEqualityConstraint(obj, y)
            % Extract state and input
            [x, u] = obj.extractStateAndControl(y);

            % Calculate the state matrices
            [x_mat, u_mat] = obj.getInputStateMatrices(x, u);

            % Calculate the constraint
            h = zeros(obj.n_state, 1);
            h(1:obj.n_x) = -x_mat(:,1) + obj.x0;
            ind_h = (obj.n_x+1):(2*obj.n_x);
            for j = 1:obj.N % Note that j = k+1
                % Calculate the constraint
                h(ind_h) = obj.Abar*x_mat(:,j)+obj.Bbar*u_mat(:,j) - x_mat(:,j+1);

                % Increment the index
                ind_h = ind_h + obj.n_x;
            end
        end

        function [A_eq, B_eq] = calculateFullEffectEquality(obj)
        %obj.calculateFullEffectEquality Calculates the equality constraint using steps
        %from the initial state and inputs to the state in question
        %
        % Inputs:
        %   P: Struct of parameters
        % 
        % Outputs: 
        %   Matrices such that A_eq*y = B_eq

            %% Calculate the state portion of the constraint
            A_eq_state = -eye(obj.n_state); % Portion corresponding to next state

            % Lower diagonal term is a block matrix of Abar matrices
            ind_row = (obj.n_x+1):(2*obj.n_x); % Row index for the matrix
            ind_col = 1:obj.n_x; % Column index for the matrix
            for k = 1:obj.N
                % Add the state update matrix
                A_eq_state(ind_row, ind_col) = obj.Abar^k;

                % Increment the indices (note, the column matrix does not update)
                ind_row = ind_row + obj.n_x;        
            end

            %% Calculate the input portion of the constraint
            A_eq_ctrl = zeros(obj.n_state, obj.n_ctrl);
            ind_row = (obj.n_x+1):(2*obj.n_x); % Row index for the matrix
            for blk_row = 1:obj.N
                % loop through columns
                ind_col = 1:obj.n_u;
                for col = (blk_row-1):-1:0
                    A_eq_ctrl(ind_row, ind_col) = obj.Abar^col*obj.Bbar;

                    % Update column indices
                    ind_col = ind_col + obj.n_u;
                end

                % Update row indices
                ind_row = ind_row + obj.n_x;
            end

            % Put the state and input portions of the matrix together
            A_eq = [A_eq_state, A_eq_ctrl];

            %% Calculate the B_eq matrix
            B_eq = zeros(obj.n_state, 1);
            B_eq(1:obj.n_x) = -obj.x0; % The first state must be equal to the initialization

        %     %% Test the full constraint
        %     u = rand(P.n_ctrl, 1)*100;
        %     x = obj.discreteSim(u, P);
        %     h = obj.stepWiseFullEffectConstraint([x; u], P)
        %     
        %     %% Test the gradient (should be commented out)
        %     % Calculate the numerical jacobian
        %     A_eq_num = jacobianest(@(y_in) obj.stepWiseFullEffectConstraint(y_in, P), rand(P.n_state+P.n_ctrl, 1)*100);
        %     
        %     % Calculate the error
        %     A_eq_err = norm(A_eq - A_eq_num, 'fro')
        end

        function h = stepWiseFullEffectConstraint(obj, y)
            % Extract state and input
            [x, u] = obj.extractStateAndControl(y);

            % Calculate the state matrices
            [x_mat, u_mat] = obj.getInputStateMatrices(x, u);

            % Calculate the constraint
            h = zeros(obj.n_state, 1);
            h(1:obj.n_x) = -x_mat(:,1) + obj.x0;
            ind_row = (obj.n_x+1):(2*obj.n_x);
            for blk_row = 1:obj.N % Note that blk_row = k+1
                % Calculate the effect on the combined state of the initial state
                x_comb = obj.Abar^blk_row*x_mat(:,1);

                % Calculate the effect on the input
                ind_u = 1;
                for exp = blk_row:-1:1
                    x_comb = x_comb + obj.Abar^(exp-1)*obj.Bbar*u_mat(:,ind_u);
                    ind_u = ind_u + 1;
                end

                % Calculate the constraint
                h(ind_row) = x_comb - x_mat(:,blk_row+1);

                % Increment the index
                ind_row = ind_row + obj.n_x;
            end
        end
    end
    
    %%% General Cost functions %%%
    methods
        function [c, dc_dy] = costCombined(obj, y)
        %obj.costCombined calculates the cost and gradient of the cost with respect to
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
            [x, u] = obj.extractStateAndControl(y);

            % Calculate the cost
            c = obj.cost(x, u);

            % Calculate the partial
            if nargout > 1 % Only calculate the partial if the calling function wants it
                dc_dy = [obj.calculatePartialWrtState(x,u), ... % dc/dx
                         obj.calculatePartialWrtInput(x,u)];    % dc/du
            else
                dc_dy = []; % Output an empty value if not needed
            end

        %     %% Check partials (Comment this code when working)
        %     % Calculate individual partials
        %     dc_dx = obj.calculatePartialWrtState(x,u,P);
        %     dc_du = obj.calculatePartialWrtInput(x,u,P);
        %     
        %     % Calculate numerical partials
        %     dc_dx_num = jacobianest(@(x_in) obj.cost(x_in, u, P), x);
        %     dc_du_num = jacobianest(@(u_in) obj.cost(x, u_in, P), u);
        %     
        %     % Calculate error
        %     dc_dx_err = norm(dc_dx - dc_dx_num, 'fro')
        %     dc_du_err = norm(dc_du - dc_du_num, 'fro')

        end

        function [c, dc_du] = costSequential(obj, u)
        %obj.costSequential calculates the cost and gradient of the cost with respect to
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
            x = obj.discreteSim(u);

            % Calcualte the cost
            c = obj.cost(x, u);

            % Calculate the partial
            if nargout > 1
                dc_du = obj.sequentialPartial(x, u);        
            else
                dc_du = [];
            end
        end

        function dc_du = sequentialPartial(obj, x, u)
        %obj.sequentialPartial: Calculates the partial of the cost wrt the input u
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
            dc_du = zeros(obj.n_u, obj.N); % We will use one column per gradient and then reshape to make a row vector at the end

            % Reshape vectors for easy access
            [x_mat, u_mat] = obj.getInputStateMatrices(x, u);

            % Initialize final lagrange multiplier (lam_N = dphi/dx(x_N)
            j = size(x_mat, 2); % Final column index corresponds to the final state
            lam_kp1 = obj.terminalPartialWrtState(x_mat(:,j))'; % dphi/dx(x_N) <-- This variable is used as lam_{k+1}
            lam_k = lam_kp1; % duplicate for initializaiton purposes (the loop moves backward 
                             % in time so at the beginning it changes iterations by
                             % setting lam_{k+1} = lam_k as k has been decremented

            % Simulate backward in time
            for j = obj.N:-1:1 % Simulate backwards in time (note that j = k+1 for matlab one indexing)
                % Extract the state and input
                uk = u_mat(:,j); % Input at iteration k
                xk = x_mat(:, j); % State at iteration k
                lam_kp1 = lam_k; % Update k index for the lagrange multipliers - \lambda at iteration k+1

                % Calculate partials needed for updates
                dLk_duk = obj.instantaneousPartialWrtControl(xk, uk, j);
                dfk_duk = obj.Bbar;
                dfk_dxk = obj.Abar;

                % Calculate partial of cost wrt state
                dLk_dxk = obj.instantaneousPartialWrtState(xk, uk, j); % Instantaneous cost does not depend on xk

                % Calculate partial
                dc_du(:, j) = (dLk_duk + lam_kp1'*dfk_duk)'; % Transposed to fit in temporary variable

                % Update Lagrange muliplier (moving backward in time)
                lam_k = dLk_dxk' + dfk_dxk'*lam_kp1;
            end

            % Reshape partial to be proper output
            dc_du = reshape(dc_du, 1, obj.n_ctrl); % changes it from partial in columns for each iteration to one single row
        end

        function dc_dx = calculatePartialWrtState(obj,x,u)
        %obj.calculatePartialWrtState Calculates the partial of the cost wrt each state
        %
        % Inputs:
        %   x: State (x_0 to x_N in a single column vector)
        %   u: Control (u_0 to u_N-1 in a single column vector) 
        %   P: Struct of parameters
        %
        % Outputs:
        %   dc_dx: partial of the cost wrt the state

            % Reformulate the states to be in terms of the column vectors
            [x_mat, u_mat] = obj.getInputStateMatrices(x, u);

            % Initialize the gradient
            ind_x = 1:obj.n_x;
            dc_dx = zeros(1, obj.n_state);
            for j = 1:obj.N % Note that j = k+1
                % Get the state error
                xk = x_mat(:,j);
                uk = u_mat(:,j);

                % Calculate the partial (assuming Q = Q')
                dc_dx(ind_x) = obj.instantaneousPartialWrtState(xk, uk, j);

                % Increment the partial index
                ind_x = ind_x + obj.n_x;
            end

            % Calculate the partial for the terminal state
            x_N = x_mat(:,end);            
            dc_dx(ind_x) = obj.terminalPartialWrtState(x_N);    
        end

        function dc_du = calculatePartialWrtInput(obj,x,u)
        %obj.calculatePartialWrtState Calculates the partial of the cost wrt each input
        %
        % Inputs:
        %   x: State (x_0 to x_N in a single column vector)
        %   u: Control (u_0 to u_N-1 in a single column vector) 
        %   P: Struct of parameters
        %
        % Outputs:
        %   dc_du: partial of the cost wrt the input

            % Reformulate the states to be in terms of the column vectors
            [x_mat, u_mat] = obj.getInputStateMatrices(x, u);

            % Initialize the gradient
            ind_u = 1:obj.n_u;
            dc_du = zeros(1, obj.n_ctrl);
            for j = 1:obj.N % Note that j = k+1
                % Get the control input
                xk = x_mat(:,j);
                uk = u_mat(:,j);

                % Calculate the partial (assuming R = R')
                dc_du(ind_u) = obj.instantaneousPartialWrtControl(xk, uk, j);

                % Increment the partial index
                ind_u = ind_u + obj.n_u;
            end
        end
    end
    
    %%% Test Functions %%%
    methods
        function testDiscreteDynamics(obj)
            % Choose a random first state and control input
            x0_test = rand(obj.n_x, 1)*100;
            u = rand(obj.n_u, 1)*10;

            % Take a continuous-time step for the sampling interval
            [tmat, xmat] = ode45(@(t, x) obj.continuousDynamics(t, x, u), [0 obj.dt], x0_test);
            xmat = xmat';

            % Take a discrete-time step for the sampling interval
            x1_disc = obj.Abar*x0_test + obj.Bbar*u;

            % Plot the continuous and discrete trajectories
            figure;
            names = {'x_1', 'x_2', 'xd_1', 'xd_2', 'xdd_1', 'xdd_2', 'xddd_1', 'xddd_2'};
            for k = 1:obj.n_x
                % Plot the continuous trajectory
                subplot(obj.n_x, 1, k);
                plot(tmat, xmat(k,:), 'b', 'linewidth', 2); hold on;

                % Plot the discrete trajectory
                plot([0 obj.dt], [x0_test(k) x1_disc(k)], 'ro', 'linewidth', 2);
                ylabel(names{k});        
            end
            xlabel('time');
        end

        function xdot = continuousDynamics(obj, ~, x, u)
            xdot = obj.A * x + obj.B*u;
        end
    end
end