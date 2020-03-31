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
            for i = 1:obj.N
                x_ind = (i-1)*obj.n_x + 1;
                u_ind = (i-1)*obj.n_u + obj.n_state + 1;
                obj.P(x_ind:x_ind+obj.n_x-1, x_ind:x_ind+obj.n_x-1) = obj.Q;
                obj.P(u_ind:u_ind+obj.n_u-1, u_ind:u_ind+obj.n_u-1) = obj.R;
            end
            obj.P(obj.n_state-obj.n_x+1:obj.n_state, obj.n_state-obj.n_x+1:obj.n_state) = obj.S;

            % get q vector (using xd and ud)
            xd = reshape(obj.calculateDesiredState(0), [], 1);
            ud = zeros(obj.n_ctrl,1);
            obj.q = -obj.P * [xd; ud];
        end
        
        function obj = calculateLinearConstraints(obj, A_c, l_c, u_c)
            %calculateLinearConstraints Calculates the full linear
            %constraint matrix A_constraints, and the lower and upper bound
            %vectors l and u.
            [A_eq, b_eq] = obj.calculateStepwiseEquality();
            obj.A_constraints = [A_eq; A_c];
            obj.l = [b_eq; l_c];
            obj.u = [b_eq; u_c];
        end
    end
    
    methods
        function [x, u] = simultaneousOptimization(obj, x0, u0)
            y0 = [x0; u0];
            obj.solver.warm_start('x', y0);
            results = obj.solver.solve();
            [x, u] = obj.extractStateAndControl(results.x);
        end
    end
end

