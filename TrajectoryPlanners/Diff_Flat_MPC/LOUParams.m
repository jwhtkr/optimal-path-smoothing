classdef LOUParams < handle
    %LOUParams A struct of parameters for the LOU static methods
    properties
        A % continuous time state matrix
        B % continuous time input matrix
        Abar % discrete time state matrix
        Bbar % discrete time state matrix
        dt = 0.1; % Discrete time interval
        n_x % Number of states
        n_u % Number of inputs
        
        % Optimization variables
        N = 50; % Number of steps
        n_ctrl % Control from u_0 to u_{N-1}
        n_state % State from x_0 to x_N
        x0 % Initial state
        
        % Optimization weights
        R = 0.1 .* diag([1, 1]); % Input squared cost (i.e. u'*R*u)
        Q = 10 .* diag([1, 1,  0, 0,   0, 0]); % state error squared (x-x_d)'Q(x-x_d)
        S = 100 .* diag([1, 1,  0, 0,   0, 0]); % state error squared (x-x_d)'S(x-x_d)
        
        % Desired variables
        xd = [] % Desired state over time
        ud = [] % Desired input over time
    end
    
    methods
        function obj = LOUParams(A, B)
            %LOUParams Construct an instance of this class
            %
            % Inputs:
            %   A: Continous-time state matrix
            %   B: Continuous-time input matrix
            
            % Store inputs
            obj.A = A;
            obj.B = B;
            
            % Process state variables
            obj.n_x = size(A, 1);
            obj.n_u = size(B, 2);
            
            % Calculate discrete dynamics
            [obj.Abar, obj.Bbar] = LOU.calculateDiscreteTimeMatrices(obj);
            
            % Initialize optimization variables
            obj.n_ctrl = obj.n_u*obj.N; % Control from u_0 to u_{N-1}
            obj.n_state = obj.n_x*(obj.N+1); % State from x_0 to x_N
            obj.x0 = zeros(obj.n_x, 1);
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

