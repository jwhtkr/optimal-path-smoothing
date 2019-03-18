classdef BetterUnicycleScenario < Scenario
    %BetterUnicycleScenario This scenario uses the better unicycle robot
        
    properties
        h_state_traj = [];
    end

    methods (Abstract)
        u = control(obj, t, x);
    end
    
    methods
        function obj = BetterUnicycleScenario()
            % Create the unicycle vehicle
            x0 = [0; 0; 0; 0; 0]; % Initial state
            veh = BetterUnicycleVehicle(x0);
            
            % Create the world
            world = EmptyWorld();
            
            % initialize the scenario
            obj = obj@Scenario(veh, world, true);            
        end
    
        %%%%  Abstract Method Implementation %%%%
        function plotState(obj, t)
            if isempty(obj.h_state_traj)
               warning('Attempting to plot uninitialized plots');
               return;
            end
            
            % Plot the state trajectory
            ind = floor(obj.getStateIndex(t));
            set(obj.h_state_traj, 'xdata', obj.xmat(1, 1:ind), 'ydata', obj.xmat(2,1:ind));
            
            % Plot the vehicle
            obj.vehicle.plotVehicle();            
        end
        
        function runScenario(obj)
            
            % Initialize plots
            obj.initializeStatePlot();
            
            % Simulate forward while plotting
            obj.integrateEuler();
            
            % Plot the results
            obj.plotResults();
        end
        
        %%%%  Plotting helper Methods %%%%
        function initializeStatePlot(obj)
            figure; 
            % Initialize the state trajectory
            obj.h_state_traj = plot(0, 0, ':b', 'linewidth', 2); hold on;
            
            % Plot the vehicle
            obj.vehicle.initializePlots(gca);
        end
        
        function plotResults(obj)
            % Create the figure and get additional data
            figure;
            
            % Plot x1 
            subplot(3,1,1);
            plot(obj.tmat, obj.xmat(1,:), 'b', 'linewidth', 2);
            ylabel('x_1(t)');
            
            % Plot x2 
            subplot(3,1,2);
            plot(obj.tmat, obj.xmat(2,:), 'b', 'linewidth', 2);
            ylabel('x_2(t)');

            % Calculate the control vs time
            ctrl = zeros(2,length(obj.tmat));
            for k = 1:length(obj.tmat)
                ctrl(:,k) = obj.control(obj.tmat(k), obj.xmat(:,k));
            end
            
            % Plot the control vs time
            subplot(3,1,3);
            plot(obj.tmat, ctrl(1,:), 'g', 'linewidth', 3); hold on;
            plot(obj.tmat, ctrl(2,:), 'b', 'linewidth', 3); 
            ylabel('Inputs');
            legend('u_v', 'u_\omega');            
            xlabel('Time (s)');
        end
        
    end
    
    
end



