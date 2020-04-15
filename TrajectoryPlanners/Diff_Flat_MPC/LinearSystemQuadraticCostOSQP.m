classdef LinearSystemQuadraticCostOSQP < LinearSystemQuadraticCost
    %LINEARSYSTEMQUADRATICCOSTOSQP Uses the OSQP solver for linear
    %quadratic programming optimization
    %   OSQP solves:
    %       min_x 1/2*x^T*P*x + q^T*x 
    %       s.t. l <= A*x <= u 
    %   where x is a vector to be minimized, P is a positive semidefinite
    %   matrix, q is a vector (where P and q encapsulate the relevant
    %   quadratic terms of the cost), and l and u are vectors representing
    %   the lower and upper bounds on the linear inequality constraints of
    %   the problem. 
    %   See the docs for OSQP at https://osqp.org/docs/index.html
    %   This class will use y instead of x because x is already defined in 
    %   the parent class. Also, A_constraints will be A, as A is already
    %   defined in the parent class.
    
    properties
        P
        q
        l
        u
        A_constraints
        solver
        
        % Additional constraints
        A_c = []
        l_c = []
        u_c = []
    end
    
    methods
        function obj = LinearSystemQuadraticCostOSQP(A, B, N, dt, des_traj, cost_mat, A_c, l_c, u_c)
            %LINEARSYSTEMQUADRATICCOSTOSQP Construct an instance of this class
            %   @param A: Continuous-time state matrix
            %   @param B: Continuous-time input matrix
            %   @param N: Number of discrete time-steps
            %   @param dt: discrete step time
            %   @param des_traj: instance of DesiredFlatTrajectory object
            %   defining the desired trajectory
            %   @param A_c: The non-dynamic constraints of the system
            %   @param l_c: The corresponding lower bound vector to A_c
            %   @param u_c: The corresponding upper bound vector to A_c
            %   @param cost_mat: struct of quadratic costs
            %            cost_mat.Q: Instantaneous cost on state
            %            cost_mat.R: Instantaneous cost on control
            %            cost_mat.S: Terminal cost on state (if empty then
            %            the infinite horizon DARE is used)
            %   A_c, l_c, and u_c can be passed in as empty arrays if not
            %   applicable to the problem.
            
            obj = obj@LinearSystemQuadraticCost(A, B, N, dt, des_traj, cost_mat);
            obj.A_c = A_c;
            obj.l_c = l_c;
            obj.u_c = u_c;
        end
        
        function obj = initializeParameters(obj)
        %initializeParameters Initializes the parameter optimization
        %parameters. In addition to the super class, this function
        %calculates the P and q parameters.
            % Call super class
            obj = initializeParameters@LinearSystemQuadraticCost(obj);
            
            obj.P = zeros(obj.n_state+obj.n_ctrl);
            for i = 1:obj.N  % For each time step
                % calc indices
                x_ind = (i-1)*obj.n_x + 1;
                u_ind = (i-1)*obj.n_u + obj.n_state + 1;
                % The block diagonal entries of P are obj.Q in the upper 
                % left (corresponding to the x portion of the y vector) and
                % obj.R in the bottom right (corresponding to the u portion
                % of the y vector).
                obj.P(x_ind:x_ind+obj.n_x-1, x_ind:x_ind+obj.n_x-1) = obj.Q;
                obj.P(u_ind:u_ind+obj.n_u-1, u_ind:u_ind+obj.n_u-1) = obj.R;
            end
            % The block diagonal entries of P corresponding to the state x
            % at the last time instance N is obj.S.
            obj.P(obj.n_state-obj.n_x+1:obj.n_state, obj.n_state-obj.n_x+1:obj.n_state) = obj.S;

            % get q vector (using xd and ud)
            obj = obj.calculateLinearCostTerm(obj.calculateDesiredState(0), ...
                obj.calculateDesiredInput(0));
            
            % Setup solver
            obj = obj.calculateLinearConstraints(obj.A_c, obj.l_c, obj.u_c);
            obj.solver = osqp;
            settings = obj.solver.default_settings();
            settings.verbose = false;
            obj.solver.setup(obj.P, obj.q, obj.A_constraints, obj.l, obj.u, settings);
        end
        
        function obj = calculateLinearConstraints(obj, A_c, l_c, u_c)
            %calculateLinearConstraints Calculates the full linear
            %constraint matrix A_constraints, and the lower and upper bound
            %vectors l and u.
            
            % get the equality constraints corresponding to the dynamics
            [A_eq, b_eq] = obj.calculateStepwiseEquality();
            % append the other linear constraints
            obj.A_constraints = [A_eq; eye(obj.n_ctrl + obj.n_state); A_c];
            % with the appropriate lower and upper bounds
            obj.l = [b_eq; obj.P_simult.lb; l_c];
            obj.u = [b_eq; obj.P_simult.ub; u_c];
        end
        
        function obj = setInitialState(obj, x0)
        %setInitialState stores the initial state
            obj = setInitialState@LinearSystemQuadraticCost(obj, x0);
            
            % Set the upper and lower bounds for the solver
            obj.l(1:obj.n_x) = -obj.x0;
            obj.u(1:obj.n_x) = -obj.x0;
            
            % Set the new constraints for the system (note: we could
            % probably do away with this if we didn't allow x0 to be a
            % variable of optimization)
            obj.solver.update('l', obj.l, 'u', obj.u);
        end
        
        function obj = updateDesiredTrajectory(obj, step)
            % Update the desired state as per the super class
            obj = updateDesiredTrajectory@LinearSystemQuadraticCost(obj, step);
            
            % Recalculate the cost term based on the desired state
            obj = obj.calculateLinearCostTerm(obj.xd, obj.ud);
            
            % Update the solver based on the desired state
            obj.solver.update('q', obj.q);
        end
        
        function obj = calculateLinearCostTerm(obj, x_des_mat, u_des_mat)
        %calculateLinearCostTerm calculates the linear portion of the cost.
        %The cost being minimized can be written as
        %   J = x'Px + q'x
        %This function calculates q which can be written as -2Pyd for this
        %cost
        %
        % Inputs:
        %   x_des_mat: n_x x N+1 matrix of state values
        %   u_des_mat: n_u x N matrix of input values
            
            % Reshape the state and input vectors
            x_des = reshape(x_des_mat, [], 1);
            u_des = reshape(u_des_mat, [], 1);
            obj.q = -obj.P*[x_des; u_des];            
        end
    end
    
    methods
        function [x, u] = simultaneousOptimization(obj, x0, u0)
            y0 = [x0; u0];
            obj.solver.warm_start('x', y0);  % warm-start with initial guess
            results = obj.solver.solve();  % the results object has lots of other info about the solution too.
            [x, u] = obj.extractStateAndControl(results.x);
        end
    end
end

