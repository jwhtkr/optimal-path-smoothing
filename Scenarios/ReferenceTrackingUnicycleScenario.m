classdef ReferenceTrackingUnicycleScenario < BetterUnicycleScenario
    %ReferenceTrackingUnicycleScenario This scenario has a unicycle robot
    %follow a sinusoidal path input
    
    properties
        % Sinusoid properties
        a = 1 % Amplitude
        f = 1 % Frequency (in radians)
        
        % State plotting variable
        h_des_point = []; % Handle to the desired point
    end
    
    methods
        function obj = ReferenceTrackingUnicycleScenario()
            % initialize the scenario
            obj = obj@BetterUnicycleScenario();            
        end
    
        %%%%  Abstract Method Implementation %%%%
        function u = control(obj, t, x)
            % Get the desired values
            [qd, qd_dot, qd_ddot] = obj.SineReference(t);
            
            u = obj.vehicle.pathControl(t, qd, qd_dot, qd_ddot, x);
        end
        
        %%%% Plotting methods - Add reference %%%%
        function plotState(obj, t)
            plotState@BetterUnicycleScenario(obj,t);
            
            if isempty(obj.h_des_point)
                warning('Attempting to plot state without initializing plots');
                return;
            end
            
            % Plot the desired position
            qd = obj.SineReference(t);
            set(obj.h_des_point, 'xdata', qd(1), 'ydata', qd(2));
        end
        
        function initializeStatePlot(obj)
            initializeStatePlot@BetterUnicycleScenario(obj);
            
            % Get and plot the desired trajectory
            qd_traj = obj.SineReference([obj.t0:obj.dt:obj.tf]);
            plot(qd_traj(1,:), qd_traj(2,:), 'r'); hold on;
            
            % Plot the desired trajectory point
            obj.h_des_point = plot(qd_traj(1,1), qd_traj(2,1), 'ro', 'linewidth', 2);
        end
        
        function plotResults(obj)
            % Create the figure and get additional data
            figure;
            qd_traj = obj.SineReference(obj.tmat);
            
            % Plot x1 and desired
            subplot(4,1,1);
            plot(obj.tmat, qd_traj(1,:), 'r', 'linewidth', 2); hold on;
            plot(obj.tmat, obj.xmat(1,:), 'b', 'linewidth', 2);
            ylabel('x_1(t)');
            legend('Desired', 'Actual');
            
            % Plot x2 and desired
            subplot(4,1,2);
            plot(obj.tmat, qd_traj(2,:), 'r', 'linewidth', 2); hold on;
            plot(obj.tmat, obj.xmat(2,:), 'b', 'linewidth', 2);
            ylabel('x_2(t)');

            % Calculate the error and control vs time
            err = zeros(1,length(obj.tmat));
            ctrl = zeros(2,length(obj.tmat));
            for k = 1:length(obj.tmat)
                err(k) = norm(qd_traj(1:2,k) - obj.xmat(1:2,k));
                ctrl(:,k) = obj.control(obj.tmat(k), obj.xmat(:,k));
            end
            
            % Plot the error vs time
            subplot(4,1,3);
            plot(obj.tmat, err, 'r', 'linewidth', 3);
            ylabel('Error');
            
            % Plot the control vs time
            subplot(4,1,4);
            plot(obj.tmat, ctrl(1,:), 'g', 'linewidth', 3); hold on;
            plot(obj.tmat, ctrl(2,:), 'b', 'linewidth', 3); 
            ylabel('Inputs');
            legend('u_v', 'u_\omega');            
            xlabel('Time (s)');
        end
        
        
        %%%% Reference trajectory methods %%%%
        function [qd, qd_dot, qd_ddot] = SineReference(obj, t)
            % Get length of the time vector
            n = length(t);
            t = reshape(t, 1, n);

            % Get the reference
            qd = [t; obj.a.*sin(obj.f.*t)];                     % Desired position
            qd_dot = [ones(1,n); obj.a*obj.f*cos(obj.f.*t)];        % Desired velocity vector
            qd_ddot = [zeros(1,n); -obj.a.*obj.f^2.*sin(obj.f.*t)]; % Desired acceleration vector
        end
    end
end

