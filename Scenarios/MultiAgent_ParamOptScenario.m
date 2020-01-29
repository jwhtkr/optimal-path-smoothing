classdef MultiAgent_ParamOptScenario < MultiScenario
    %VelocityTrackingUnicycleScenario This scenario has a robot
    %track desired velocities
    
    properties
        planner;
        % Desired velocity values
        v_d; % Desired translational velocity
        w_d; % Desired rotational velocity
        u_init;
    end
    
    methods
        function obj = MultiAgent_ParamOptScenario(n_agents,world,leaderVehicle)
            leader = virtual_leader(leaderVehicle,n_agents);
            
            for k= 1:n_agents
                q = leader.getDesiredFollowerPosition(k);
                theta = leader.vehicle.x(leader.vehicle.th_ind);
                agents(k) = agent(BetterUnicycleVehicle([q; theta; 1; 0]),n_agents,k);
            end

            
            
            obj = obj@MultiScenario(leader, agents, world, true);
            
            obj.v_d = 1;
            obj.w_d = 0.0;
            obj.vl.vehicle.u_init = [obj.v_d, obj.w_d, obj.vl.agent.planner.T/3, obj.v_d, obj.w_d, obj.vl.agent.planner.T/3*2, obj.v_d, obj.w_d]';
            obj.vl.setLeaderTerminalState(obj.vl.vehicle.x, obj.vl.vehicle.u_init);
            
            for k= 1:n_agents
                obj.agents(k).vehicle.v_d = obj.v_d;
                obj.agents(k).vehicle.w_d = obj.w_d;
                obj.agents(k).planner.qd = obj.vl.getDesiredFollowerGoal(k);
                obj.agents(k).vehicle.u_init = [obj.v_d, obj.w_d, obj.agents(k).planner.T/3, obj.v_d, obj.w_d, obj.agents(k).planner.T/3*2, obj.v_d, obj.w_d]';
                obj.agents(k).vl = obj.vl;
            end
            
            

        end
    
        %%%%  Abstract Method Implementation %%%%
        function u = control(obj, t, x)
            % Calculate VL Control
%             u = obj.vl.planner.minimize(x,obj.vl.vehicle.u_init);
            obj.setDesiredVelocities(x, t);
            obj.vl.vehicle.v_d = obj.v_d; %u(obj.vl.planner.ind_a1);
            obj.vl.vehicle.w_d = obj.w_d; %u(obj.vl.planner.ind_alpha1);
%             obj.vl.vehicle.u_init = u;
            
            
            
            % Calculate the velocity control            
            u = obj.vl.vehicle.velocityControl(obj.vl.vehicle.v_d, obj.vl.vehicle.w_d, x);
            obj.vl.setLeaderTerminalState(obj.vl.vehicle.x, obj.vl.vehicle.u_init);
            
            % Calculate Agent Control
            for k = 1:obj.n_agents
                tic
                obj.agents(k).planner.qd = obj.vl.getDesiredFollowerGoal(k);
                obj.agents(k).planner.leader_traj = obj.vl.leader_traj;
                [xo,yo,do] = obj.agents(k).vehicle.getObstacleDetections(obj.world);
                obj.agents(k).planner.setObstacles(xo,yo,do);
                % method to calculate desired velocities

                u_agent = obj.agents(k).planner.minimize(obj.agents(k).vehicle.x,obj.agents(k).vehicle.u_init);
                obj.agents(k).vehicle.v_d = u_agent(obj.agents(k).planner.ind_a1);
                obj.agents(k).vehicle.w_d = u_agent(obj.agents(k).planner.ind_alpha1);
                obj.agents(k).vehicle.u_init = u_agent;
                u(:,k+1) = obj.agents(k).vehicle.velocityControl(obj.agents(k).vehicle.v_d, obj.agents(k).vehicle.w_d, obj.agents(k).vehicle.x);
                toc
            end
        end
        
        function setDesiredVelocities(obj, x, t)
            if x(1) >= 7.5
                obj.v_d = 1;
                obj.w_d = .29;
            end
            if x(2) >= 3.0
                obj.v_d = 1;
                obj.w_d = -.29;
            end
            if x(1) >= 14.25
                obj.v_d = 1;
                obj.w_d = 0.0;
            end
            if x(1) >= 20
                obj.v_d = 0.0;
                obj.w_d = 0.0;
            end 
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
                ctrl(:,k) = obj.control(obj.tmat(k), obj.xmat(:,k));
                [v_vec(k), w_vec(k)] = obj.vl.vehicle.kinematics.getVelocities(obj.tmat(k), obj.xmat(:,k), ctrl(:,k));
            end
            
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

