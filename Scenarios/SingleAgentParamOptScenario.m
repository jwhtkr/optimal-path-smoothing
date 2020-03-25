classdef SingleAgentParamOptScenario < Scenario
    %VelocityTrackingUnicycleScenario This scenario has a robot
    %track desired velocities
    
    properties
        planner;
        % Desired velocity values
        v_d; % Desired translational velocity
        w_d; % Desired rotational velocity
        u_init;
        trajectory;
        video;
        cost;
        L;
        dphi_dx;
        f;
        
        
    end
    
    methods
        function obj = SingleAgentParamOptScenario(world, vehicle, path)

            obj = obj@Scenario(vehicle, world, true);
            
            %Initialize agent
            obj.trajectory = TrajUtil.createClothoidTrajectory(path, 1, obj.dt, .85, .85);
%             obj.trajectory = TrajUtil.createClothoidTrajectory(path, 1, .0005, .85, .85);
            obj.planner = Unicycle2(obj.vehicle.x);
            obj.planner.trajectory = obj.trajectory;
            obj.tf = (size(obj.trajectory.x,2)-1)*obj.dt - obj.planner.tf - obj.dt;
            obj.vehicle.u_init = [0; 0; 0; 0; 0; 0; 0; 0; 0];
            obj.v_d = 1;
            obj.w_d = 0.0;
        end
    
        %%%%  Abstract Method Implementation %%%%
        function u = control(obj, t, x)
            
            obj.planner.xd = obj.trajectory.stateAtTime(t+obj.planner.tf); %obj.leader.getDesiredFollowerGoal(k);
            
% %             obj.planner.leader_traj = obj.leader.trajectory;
%             [xo,yo,do] = obj.vehicle.getObstacleDetections(obj.world);
%             obj.planner.setObstacles(xo,yo,do);
%             % method to calculate desired velocities
% 
%             u_agent = obj.planner.minimize(t,obj.vehicle.x,obj.vehicle.u_init);
%             obj.vehicle.v_d = u_agent(obj.planner.ind_v1);
%             obj.vehicle.w_d = u_agent(obj.planner.ind_w1);
%             if u_agent(obj.planner.ind_time1) == 0 
%                 obj.vehicle.v_d = u_agent(obj.planner.ind_v2);
%                 obj.vehicle.w_d = u_agent(obj.planner.ind_w2);
%                 if u_agent(obj.planner.ind_time2) == 0 
%                     obj.vehicle.v_d = u_agent(obj.planner.ind_v3);
%                     obj.vehicle.w_d = u_agent(obj.planner.ind_w3);
%                 end
%             end
%             obj.vehicle.u_init = u_agent;
%             if u_agent(obj.planner.ind_time1) == 0 && u_agent(obj.planner.ind_time2) == 0 && u_agent(obj.planner.ind_time3) == 0 
%                 u = obj.trackControl(t);
%             else
%                 u = obj.vehicle.velocityControl(obj.vehicle.v_d, obj.vehicle.w_d, obj.vehicle.x);
%             end

            u_agent = [0;0;0;0;0;0;0;0;0];
            obj.planner.reset(x);
            obj.planner.t_sim = t;
            obj.dphi_dx = [obj.dphi_dx; obj.planner.terminalStatePartial(x)];
            obj.f = [obj.f obj.planner.unicycleDualModeDynamics(obj.planner.dt, x, u_agent) - obj.trajectory.xdotAtTime(t)];
            obj.L = [obj.L; obj.planner.instantaneousCost(obj.planner.dt, x, u_agent)];
%             cost = obj.planner.cost([0;0;0;0;0;0;0;0;0])
            obj.cost = [obj.cost; obj.planner.cost(u_agent)];
            u = obj.trackControl(t);

        end
        
        function u = trackControl(obj, t)
            traj = @(t) obj.trajectory.reference_traj(t);
            u = obj.vehicle.TrackTrajectoryApproximateDiffeomorphism(t, obj.vehicle.x, traj);
        end

        
        %%%% Plotting methods - Add desired velocities %%%%
        function plotResults(obj)
            % Create the figure and get additional data
            figure;
            
            
            % Calculate the control and velocities vs time
            t_len = length(obj.tmat);
            v_vec = zeros(1, t_len);
            w_vec = zeros(1, t_len);
%             vd_vec = zeros(1, t_len);
%             wd_vec = zeros(1, t_len);
            xd_vec = zeros(1, t_len);
            yd_vec = zeros(1, t_len);
            err_vec = zeros(1, t_len);
            yaw_err_vec = zeros(1, t_len);
            for k = 1:t_len
%                 u = obj.control(obj.tmat(k), obj.xmat(:,k));
%                 ctrl(:,k) = u(:,1);
                xd_vec(:,k) = obj.trajectory.x(k);
                yd_vec(:,k) = obj.trajectory.y(k);
                err_vec(:,k) = norm([obj.trajectory.x(k);obj.trajectory.y(k)] - obj.xmat(1:2,k));
%                 vd_vec(:,k) = obj.trajectory.v(k);
%                 wd_vec(:,k) = obj.trajectory.w(k);
                h_e = [cos(obj.trajectory.psi(k));sin(obj.trajectory.psi(k))] - [cos(obj.xmat(3,k));sin(obj.xmat(3,k))];
                yaw_err_vec(:,k) = h_e'*h_e/2;
                dv1(k) = obj.dphi_dx(k,:)*obj.f(:,k);
                dv2(k) = obj.dphi_dx(k,:)*obj.f(:,k) + obj.L(k);
                
                
                [v_vec(:,k), w_vec(:,k)] = obj.vehicle.kinematics.getVelocities(obj.tmat(k), obj.xmat(:,k), obj.ctrl(:,k));
                err_v_vec(k) = obj.trajectory.v(k) - v_vec(:,k);
                err_w_vec(k) = obj.trajectory.w(k) - w_vec(:,k);
            end
            subplot(2,1,1) 
            plot(obj.tmat, obj.cost(:), 'b', 'linewidth', 2); hold on;
            plot(obj.tmat, yaw_err_vec(1,:), ':r', 'linewidth', 2);
            ylabel('Cost');
            legend('Cost','Yaw Error');
            
            subplot(2,1,2) 
            plot(obj.tmat, dv1(:), 'b', 'linewidth', 2); hold on;
            plot(obj.tmat, dv2(:), 'r', 'linewidth', 2);
            ylabel('$\frac{dphi}{dx}f(x,\kappa_N)$','Interpreter','latex');
            legend('no instant', 'instant')
            
            % Plot x and desired
%             subplot(5,1,1);
%             plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
%             plot(obj.tmat, obj.xmat(1,:) - xd_vec(1,:), 'b', 'linewidth', 2);
%             ylabel('x(t) Error');
%             legend('Desired', 'Actual');
            
            % Plot y and desired
%             subplot(5,1,2);
%             plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
%             plot(obj.tmat, obj.xmat(2,:) - yd_vec(1,:), 'b', 'linewidth', 2);
%             ylabel('y(t) Error');
%             legend('Desired', 'Actual');
            
            %Plot Error
            figure
            subplot(4,1,1);
            plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
            plot(obj.tmat, err_vec(1,:), 'b', 'linewidth', 2);
            ylabel('Position Error');
%             legend('Desired', 'Actual');
            
            %Plot Error
            subplot(4,1,2);
            plot([obj.tmat(1) obj.tmat(end)],[0 0], ':r', 'linewidth', 2); hold on;
            plot(obj.tmat, yaw_err_vec(1,:), 'b', 'linewidth', 2);
            ylabel('Yaw Error');
%             legend('Desired', 'Actual');
            
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
%             legend('Desired', 'Actual');

%             % Plot the control vs time
%             subplot(5,1,5);
%             plot(obj.tmat, obj.ctrl(1,:), 'g', 'linewidth', 3); hold on;
%             plot(obj.tmat, obj.ctrl(2,:), 'b', 'linewidth', 3); 
%             ylabel('Inputs');
%             legend('u_v', 'u_\omega');
%             xlabel('Time (s)');
        end
        
        function initializeStatePlot(obj)
            figure; 
            % Plot the world
            obj.world.plotWorld(gca);
            
            % Initialize the state trajectory
            obj.h_state_traj = plot(0, 0, ':b', 'linewidth', 2); hold on;
            
            % Plot the vehicle
            obj.vehicle.initializePlots(gca);
            
            plot(obj.trajectory.x,obj.trajectory.y,'g');
            pause();
        end
        
        
    end
end

