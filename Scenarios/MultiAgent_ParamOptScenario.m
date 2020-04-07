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
            v_vec = zeros(1, t_len);
            w_vec = zeros(1, t_len);
            vd_vec = zeros(1, t_len);
            wd_vec = zeros(1, t_len);
            xd_vec = zeros(1, t_len);
            yd_vec = zeros(1, t_len);
            err_vec = zeros(1, t_len);
            yaw_err_vec = zeros(1, t_len);
            for k = 1:t_len
                u = obj.ctrl(2*k-1:2*k,1);
                [v_vec(:,k), w_vec(:,k)] = obj.leader.vehicle.kinematics.getVelocities(obj.tmat(k), obj.xmat(:,k), u(:,1));
                err_v_vec(k) = obj.leader.trajectory.v(k+1) - v_vec(:,k);
                err_w_vec(k) = obj.leader.trajectory.w(k+1) - w_vec(:,k);
                
                xd_vec(:,k) = obj.leader.trajectory.x(k+1);
                yd_vec(:,k) = obj.leader.trajectory.y(k+1);
                err_vec(:,k) = norm([obj.leader.trajectory.x(k+1);obj.leader.trajectory.y(k+1)] - obj.xmat(1:2,k));
                
                h_e = [cos(obj.leader.trajectory.psi(k+1));sin(obj.leader.trajectory.psi(k+1))] - [cos(obj.xmat(3,k));sin(obj.xmat(3,k))];
                yaw_err_vec(:,k) = h_e'*h_e/2;
            end
            
            %Plot Error
            figure
            subplot(4,1,1);
            plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
            plot(obj.tmat, err_vec(1,:), 'b', 'linewidth', 2);
            ylabel('Position Error');
            legend('Desired', 'Actual');
            
            %Plot Error
            subplot(4,1,2);
            plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
            plot(obj.tmat, yaw_err_vec(1,:), 'b', 'linewidth', 2);
            ylabel('Yaw Error');
            legend('Desired', 'Actual');
            
            % Plot v and desired
            subplot(4,1,3);
            plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
            plot(obj.tmat, err_v_vec, 'b', 'linewidth', 2);
            ylabel('Vel Error');
            %             legend('Desired', 'Actual');
            
            % Plot w and desired
            subplot(4,1,4);
            plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
            plot(obj.tmat, err_w_vec, 'b', 'linewidth', 2);
            ylabel('Rotational Vel Error');
            
            
            
            %Plot Error for Agents
            for i = 1:obj.n_agents
                t_len = length(obj.tmat);
                v_vec = zeros(1, t_len);
                w_vec = zeros(1, t_len);
                xd_vec = zeros(1, t_len);
                yd_vec = zeros(1, t_len);
                err_vec = zeros(1, t_len);
                yaw_err_vec = zeros(1, t_len);
                for k = 1:t_len
                    u = obj.ctrl(2*k-1:2*k,1);
                    [v_vec(:,k), w_vec(:,k)] = obj.agents(i).vehicle.kinematics.getVelocities(obj.tmat(k), obj.xmat(:,k), u(:,1));
                    err_v_vec(k) = obj.agents(i).trajectory.v(k+1) - v_vec(:,k);
                    err_w_vec(k) = obj.agents(i).trajectory.w(k+1) - w_vec(:,k);
                    
                    xd_vec(:,k) = obj.agents(i).trajectory.x(k+1);
                    yd_vec(:,k) = obj.agents(i).trajectory.y(k+1);
                    err_vec(:,k) = norm([obj.agents(i).trajectory.x(k+1);obj.agents(i).trajectory.y(k+1)] - obj.xmat(1:2,k));

                    h_e = [cos(obj.agents(i).trajectory.psi(k+1));sin(obj.agents(i).trajectory.psi(k+1))] - [cos(obj.xmat(3,k));sin(obj.xmat(3,k))];
                    yaw_err_vec(:,k) = h_e'*h_e/2;
                end
                
                %Plot Error
                figure
                subplot(4,1,1);
                plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
                plot(obj.tmat, err_vec(1,:), 'b', 'linewidth', 2);
                ylabel('Position Error');
                legend('Desired', 'Actual');
                
                %Plot Error
                subplot(4,1,2);
                plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
                plot(obj.tmat, yaw_err_vec(1,:), 'b', 'linewidth', 2);
                ylabel('Yaw Error');
                legend('Desired', 'Actual');
                
                % Plot v and desired
                subplot(4,1,3);
                plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
                plot(obj.tmat, err_v_vec, 'b', 'linewidth', 2);
                ylabel('Vel Error');
                %             legend('Desired', 'Actual');
                
                % Plot w and desired
                subplot(4,1,4);
                plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
                plot(obj.tmat, err_w_vec, 'b', 'linewidth', 2);
                ylabel('Rotational Vel Error');
            end
        end
        
    end
end

