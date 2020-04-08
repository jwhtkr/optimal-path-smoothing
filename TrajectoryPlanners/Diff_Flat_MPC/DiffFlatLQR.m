classdef DiffFlatLQR < handle
    %DiffFlatLQR implments a time varying control law to follow a
    %trajectory produced by a differentially flat input
    
    properties(Access=protected)
        df_traj = ExactTrajRep4(); % Stores the differentially flat trajectory
        T = 5; % Final time for control
        P_0 % Stores the initial value for the DRE
        
        % Cost values for LQR problem
        Q = 0.1 .* eye(5);
        R = 0.1 .* eye(2);
        R_inv % Inverse of R
        S = diag([1, 1, 0, 0, 0]);
        
        % State matrices
        B = [zeros(3,2), eye(2)]; % Linearized input matrix
        A_const
        n_x = 5;
        n_u = 2;
    end
    
    methods
        function obj = DiffFlatLQR()
            %DiffFlatLQR Construct an instance of this class            
            obj.R_inv = inv(R);
            obj.A_const = zeros(obj.n_x);
            obj.A_const(3,1) = 1;
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
        
        function P = calculatePLinSys(obj, t)
        %calculatePLinSys Calculates the solution to the DRE at time t <
        %obj.T using a combination of linear systems
            % Use Euler integration to integrate backwards in time
            X = eye(obj.n_x);
            Y = obj.S;
            dt = 0.01;
            
            % Start from the terminal time and integrate backwards
            t = obj.T;
            while t 
        end
    end
    
    methods(Access=protected)
        
    end
end

