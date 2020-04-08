classdef ExactTrajRep4 < handle
    %ExactTrajRep4 Represents the extact representation for the
    %differentially flat system defined by 
    % \dot{x} = Ax + Bu where
    %   A = [Z I Z Z; 
    %        Z Z I Z; 
    %        Z Z Z I; 
    %        Z Z Z Z]; % state matrix
    %
    %   B = [Z; Z; Z; I]; % Input matrix
    
    properties
        % Dynamic matrices
        A %[Z I Z Z; Z Z I Z; Z Z Z I; Z Z Z Z]; % state matrix
        B %[Z; Z; Z; I]; % Input matrix
        
        % Solution
        dt % Spacing interval
        x_mat % 8x(N+1) matrix of states
        u_mat % 2xN matrix of inputs
        t1 % time of x(1), u(1)
        N % Number of steps
    end
    
    properties(Constant)
        n_x = 8 % Number of states at a given time
        n_u = 2 % Number of inputs at a given time
    end
    
    methods
        function obj = ExactTrajRep4()
            % Create the continuous time linear system
            Z = zeros(2); % 2x2 matrix of zeros
            I = eye(2); % 2x2 identity matrix
            obj.A = [Z I Z Z; Z Z I Z; Z Z Z I; Z Z Z Z]; % state matrix
            obj.B = [Z; Z; Z; I]; % Input matrix 
        end
        
        function setValues(obj, x_in, u_in, dt, t1)
        %setValues Stores the values of the differentially flat system
        %
        % Inputs:
        %   x_in: states, can be either a column vector or 8x(N+1) matrix
        %   u_in: controls, can be either a column vector a 2xN matrix
        %   dt: time spacing
        %   t1: time of x(1)
            % Store the inputs
            obj.dt = dt;
            obj.t1 = t1;
            obj.x_mat = reshape(x_in, obj.n_x, []);
            obj.u_mat = reshape(u_in, obj.n_u, []);
            obj.N = size(obj.u_mat, 2);
        end
        
        function [x, u] = getStateAndControl(obj, t)
        %getStateAndControl Returns the state and control for the given
        %time
        %
        % Inputs:
        %   t: Time to be evaluated
        % 
        % Outputs:
        %   x: state at time t
        %   u: input at time t
        
            % Calculate the index
            delta_t1 = t - obj.t1; % Change of time from beginning
            step = floor(delta_t1/obj.dt) + 1; % index of the time before
            
            % Ensure that step is valid
            assert(step > 0);
            step = min(step, obj.N); % Make step correspond to the final control
                        
            % Calculate the time of the step
            step_t = (step-1)*obj.dt; % time of x(step)
            delta = t - step_t; % The difference between the previous state and t
            
            % Extract the state and input at the given step
            x_step = obj.x_mat(:,step);
            u = obj.u_mat(:,step);
            
            % Calculate the matrices used for calculating the new state
            eAdelta = expmA(delta);
            Xdelta = int_expmA(delta);
            
            % Calculate the updated state
            x = eAdelta*x_step + Xdelta*obj.B*u;            
        end
        
        function plotComparison(obj)
        %plotComparison plots the results of interpolating using
        %getStateAndControl
            % Loop through and calculate the state
            dt_hr = obj.dt ./ 10; % Time spacing in high resolution
            t = obj.t1:dt_hr:(obj.t1+obj.N*obj.dt);
            x_mat_hr = zeros(obj.n_x, length(t));
            for k = 1:length(t)
                [x_mat_hr(:,k), ~] = obj.getStateAndControl(t(k));
            end
            
            % Calcualte the time matrix
            t_mat = obj.t1;
            for k = 1:obj.N
                t_mat = [t_mat t_mat(end)+obj.dt];
            end
            
            % Plot the results
            figure('units','normalized','outerposition',[0 0 1 1]);
            for k = 1:obj.n_x
                subplot(4, 2, k);
                plot(t, x_mat_hr(k,:), 'r', 'linewidth', 2); hold on;
                plot(t_mat, obj.x_mat(k,:), 'bo');
                ylabel(['x_' num2str(k)]);
            end
        end
    end
    
    methods(Static)
        function test_int_expmA()
            X0 = zeros(64,1);
            tf = 100 + rand;
            
            tic
            [tmat, xmat] = ode45(@(t, x) dynamics(t, x), [0 tf], X0);
            time_sim = toc
            xmat = xmat';
            
            % Get the final state
            xf = xmat(:,end);
            Xf = reshape(xf, 8,8);
            
            % Compare to integration solution
            tic
            Xf_int = int_expmA(tf);
            time_calc = toc
            err = norm(Xf-Xf_int, 'fro')
            
            function xdot = dynamics(t, x)
                X = reshape(x, 8, 8);
                Xdot = expmA(t);
                xdot = reshape(Xdot, 64, 1);
            end
        end
    end
end

function eAt = expmA(t)
%expmA returns expm(A*t)
    % Precompute the exponentials
    t_2_2 = t^2/2;
    t_3_6 = t^3/6;
    
    % Compute the matrix exponential
    eAt = [ 1, 0, t, 0, t_2_2,     0, t_3_6,     0;...
            0, 1, 0, t,     0, t_2_2,     0, t_3_6;...
            0, 0, 1, 0,     t,     0, t_2_2,     0;...
            0, 0, 0, 1,     0,     t,     0, t_2_2;...
            0, 0, 0, 0,     1,     0,     t,     0;...                  
            0, 0, 0, 0,     0,     1,     0,     t;...
            0, 0, 0, 0,     0,     0,     1,     0;...
            0, 0, 0, 0,     0,     0,     0,     1];
end

function X = int_expmA(t)
%int_expmA Integrates expm(A\tau) from \tau = 0 to t
    % Precompute the exponentials
    t_2_2 = t^2/2;
    t_3_6 = t^3/6;
    t_4_24 = t^4/24;
    
    % Calculate the integration
    X   = [ t, 0, t_2_2,     0,     t_3_6,         0,    t_4_24,      0;...
            0, t,     0, t_2_2,         0,     t_3_6,         0, t_4_24;...
            0, 0,     t,     0,     t_2_2,         0,     t_3_6,      0;...
            0, 0,     0,     t,         0,     t_2_2,         0,  t_3_6;...
            0, 0,     0,     0,         t,         0,     t_2_2,      0;...                  
            0, 0,     0,     0,         0,         t,         0,  t_2_2;...
            0, 0,     0,     0,         0,         0,         t,      0;...
            0, 0,     0,     0,         0,         0,         0,      t];
end

