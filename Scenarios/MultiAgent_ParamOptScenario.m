classdef MultiAgent_ParamOptScenario < MultiScenario
    %VelocityTrackingUnicycleScenario This scenario has a robot
    %track desired velocities
    
    properties
        planner;
        % Desired velocity values
        v_d; % Desired translational velocity
        w_d; % Desired rotational velocity
        u_init;
        ctrl = [];
    end
    
    methods
        function obj = MultiAgent_ParamOptScenario(n_agents, world, leaderVehicle, path)

            obj = obj@MultiScenario(leaderVehicle, n_agents, world, true, path);
            
            %Initialize leader and agents velocities and goals
            obj.v_d = 1;
            obj.w_d = 0.0;
        end
    
        %%%%  Abstract Method Implementation %%%%
        function u = control(obj, t, x)  
            % Calc Leader Control
            u = obj.leader.trackControl(t);
            
            % Calculate Agent Control
            for k = 1:obj.n_agents
                tic
                u(:,k+1) = obj.agents(k).MPC_output(t, obj.world);
%                 u(:,k+1) = obj.agents(k).trackControl(t);
                toc
            end
            obj.ctrl = [obj.ctrl; u];
        end

        
        %%%% Plotting methods - Add desired velocities %%%%
        function plotResults(obj)
            % Create the figure and get additional data
            figure;
            
            
            % Calculate the control and velocities vs time
            t_len = length(obj.tmat);
            ctrl = zeros(2,t_len);
            v_vec = zeros(1, t_len);
            w_vec = zeros(1, t_len);
            for k = 1:t_len
                u = obj.ctrl(2*k:2*k+1,1);
                ctrl(:,k) = u;
                [v_vec(:,k), w_vec(:,k)] = obj.leader.vehicle.kinematics.getVelocities(obj.tmat(k), obj.xmat(:,k), u(:,1));
                %%% TODO: final Plots for each agent
%                 for i = 1:obj.n_agents
%                     [v_vec(i,k), w_vec(i,k)] = obj.leader.vehicle.kinematics.getVelocities(obj.tmat(k), obj.xmat(:,k), u(:,1));
%                 end
            end
            
            % Plot Error for Leader
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
            
            %Plot Error for Agents
            for i = 1:obj.n_agents
                t_len = length(obj.tmat);
                ctrl = zeros(2,t_len);
                v_vec = zeros(1, t_len);
                w_vec = zeros(1, t_len);
                for k = 1:t_len
                    u = obj.ctrl(2*k:2*k+1,i+1);
                    ctrl(:,k) = u;
                    [v_vec(:,k), w_vec(:,k)] = obj.agent(i).vehicle.kinematics.getVelocities(obj.tmat(k), obj.xmat(:,k), u(:,1));
                end
            end
        end
        
    end
end

