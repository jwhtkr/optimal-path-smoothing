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
    end
    
    methods
        function obj = LinearSystemQuadraticCostOSQP(A, B, N, A_c, l_c, u_c)
            %LINEARSYSTEMQUADRATICCOSTOSQP Construct an instance of this class
            %   @param A: Continuous-time state matrix
            %   @param B: Continuous-time input matrix
            %   @param N: Number of discrete time-steps
            %   @param A_c: The non-dynamic constraints of the system
            %   @param l_c: The corresponding lower bound vector to A_c
            %   @param u_c: The corresponding upper bound vector to A_c
            %
            %   A_c, l_c, and u_c can be passed in as empty arrays if not
            %   applicable to the problem.
            
            obj = obj@LinearSystemQuadraticCost(A, B, N);
            obj = obj.calculateLinearConstraints(A_c, l_c, u_c);
            obj.solver = osqp;
            obj.solver.setup(obj.P, obj.q, obj.A_constraints, obj.l, obj.u);
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
            xd = reshape(obj.calculateDesiredState(0), [], 1);
            ud = zeros(obj.n_ctrl,1);  % this is a short-cut way of getting
                                       % zeros in q corresponding to the u 
                                       % portion of y
            obj.q = -obj.P * [xd; ud];
        end
        
        function obj = calculateLinearConstraints(obj, A_c, l_c, u_c)
            %calculateLinearConstraints Calculates the full linear
            %constraint matrix A_constraints, and the lower and upper bound
            %vectors l and u.
            
            % get the equality constraints corresponding to the dynamics
            [A_eq, b_eq] = obj.calculateStepwiseEquality();
            % append the other linear constraints
            obj.A_constraints = [A_eq; A_c];
            % with the appropriate lower and upper bounds
            obj.l = [b_eq; l_c];
            obj.u = [b_eq; u_c];
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

