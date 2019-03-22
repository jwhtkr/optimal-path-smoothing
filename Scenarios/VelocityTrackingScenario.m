classdef VelocityTrackingScenario < Scenario
    %VelocityTrackingUnicycleScenario This scenario has a unicycle robot
    %track desired velocities
    
    properties
        % Desired velocity values
        v_d = 5 % Desired translational velocity
        w_d = 0.5 % Desired rotational velocity
    end
    
    methods
        function obj = VelocityTrackingScenario(veh)
            % Create the world
            world = EmptyWorld();
            
            % initialize the scenario
            obj = obj@Scenario(veh, world, true);           
        end
    
        %%%%  Abstract Method Implementation %%%%
        function u = control(obj, t, x)
            % Calculate the velocity control            
            u = obj.vehicle.velocityControl(obj.v_d, obj.w_d, x);
        end
        
        %%%% Plotting methods - Add desired velocities %%%%
        function plotResults(obj)
            % Create the figure and get additional data
            figure;
            
            
            % Calculate the control and velocities vs time
            ctrl = zeros(2,length(obj.tmat));
            for k = 1:length(obj.tmat)
                ctrl(:,k) = obj.control(obj.tmat(k), obj.xmat(:,k));
            end
            v_vec = obj.xmat(obj.vehicle.kinematics.v_ind, :);
            w_vec = obj.xmat(obj.vehicle.kinematics.w_ind, :);
            
            % Plot v and desired
            subplot(3,1,1);
            plot([obj.tmat(1) obj.tmat(end)],[obj.v_d obj.v_d], ':r', 'linewidth', 2); hold on;
            plot(obj.tmat, v_vec, 'b', 'linewidth', 2);
            ylabel('v(t)');
            legend('Desired', 'Actual');
            
            % Plot w and desired
            subplot(3,1,2);
            plot([obj.tmat(1) obj.tmat(end)],[obj.w_d obj.w_d], ':r', 'linewidth', 2); hold on;
            plot(obj.tmat, w_vec, 'b', 'linewidth', 2);
            ylabel('\omega(t)');

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

