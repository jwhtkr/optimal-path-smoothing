classdef virtual_leader < handle
    % This class implements a virtual leader
    
    properties (SetAccess = public, GetAccess = public)
        s  = 0;             % Current index into reference trajectory
        sdot             % Time derivative of s
        
        v0              % Leader velocity
        
        % Variables for dynamics of s
        feedforward
        Vu             % Upper bound on Lyapunov function
        delta =.1           % Some scalar
        
        % Plotting variables
        v_current = 0;
        t_current = 0;
        v_h
        s_h
        v_nom_h
        s_nom_h
        ax_v
        ax_s
        fontsize = 12;
        
        vehicle;
        n_agents;
        agent_position;
        voronoiBarrier;
        leader_xT;
        
        t_span;
        leader_traj;
        agent;
        agent_num;
        Q;
        V;
        trajectory;
        path;
    end
    
    methods
        function obj = virtual_leader(vehicle, n_agents)
            obj.vehicle = vehicle;
            obj.n_agents = n_agents;
            obj.agent_num = 0;
            obj.agent = agent(vehicle, n_agents, obj.agent_num);
        end       
        
        % Might not need
        function q_fd = getDesiredFollowerPosition(obj, agent_num)
            obj.updateFormation();
            q_fd = obj.Q(:,agent_num);
        end
        
        % Might not need
        function q_fd = getDesiredFollowerGoal(obj, agent_num)
            q_l = obj.leader_xT(obj.vehicle.q_ind);
            theta = obj.leader_xT(obj.vehicle.th_ind);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            
            q_fd = q_l + R*obj.agent.Q(:,agent_num);
        end
        
        % might not need
        function setLeaderTerminalState(obj,x,u)
            [obj.leader_traj, obj.leader_xT] = obj.agent.planner.getTerminalPosition(x,u);
            obj.t_span = obj.agent.planner.t_span;
        end
        
        function initializePlots(obj, ax)
            obj.vehicle.sensor.initializePlots(ax);
            obj.vehicle.kinematics.c = 'r';
            obj.vehicle.kinematics.initializeStatePlot(ax, obj.vehicle.x);
            plot(obj.trajectory.x,obj.trajectory.y);
            if ~(obj.n_agents ==0)
                obj.updateFormation();

                obj.agent_position = plot(ax, obj.Q(1,:), obj.Q(2,:), 'ko', 'linewidth', 3);

                for k = 1:length(obj.V)/2
                    obj.voronoiBarrier(k) = plot(ax, obj.V(1,2*k-1:2*k), obj.V(2,2*k-1:2*k),'k');
                end
            end
        end
        
        function plotFormation(obj)
            obj.vehicle.plotVehicle();
            if ~(obj.n_agents ==0)
                obj.updateFormation();

                set(obj.agent_position, 'xdata', obj.Q(1,:), 'ydata', obj.Q(2,:));
                for k = 1:length(obj.V)/2               
                    set(obj.voronoiBarrier(k), 'xdata', obj.V(1,2*k-1:2*k), 'ydata', obj.V(2,2*k-1:2*k));
                end 
            end
        end
        
        
        function updateFormation(obj)
            q_l = obj.vehicle.x(obj.vehicle.q_ind);
            theta = obj.vehicle.x(obj.vehicle.th_ind);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            
            obj.Q = R*obj.agent.Q + q_l*ones(1,obj.n_agents);
            obj.V = R*obj.agent.planner.voronoi.V + q_l*ones(1,length(obj.agent.planner.voronoi.V));
        end
        
        function u = trackControl(obj, t, x)
            traj = @(t) obj.trajectory.reference_traj(t);
            u = obj.vehicle.TrackTrajectoryApproximateDiffeomorphism(t, x, traj);
        end
        
        function traj = getFollowerTrajectory(obj,agent_num)
            traj_leader = obj.trajectory;
            q_leader = [traj_leader.x; traj_leader.y];
            qdot_leader = [traj_leader.xdot; traj_leader.ydot];
            qddot_leader = [traj_leader.xddot; traj_leader.yddot];
            qdddot_leader = [traj_leader.xdddot; traj_leader.ydddot];
            q_fd = obj.Q(:,agent_num);
            
            for k = 1:length(traj_leader.psi)
                R = [cos(traj_leader.psi(k)), -sin(traj_leader.psi(k)); sin(traj_leader.psi(k)), cos(traj_leader.psi(k))];
                w_hat = [0 -traj_leader.w(k); traj_leader.w(k) 0];
                alpha_hat = [0 -traj_leader.alpha(k); traj_leader.alpha(k) 0];
                zeta_hat = [0 -traj_leader.zeta(k); traj_leader.zeta(k) 0];
                
                q_follower(:,k) = R*q_fd + q_leader(:,k);
                qdot_follower(:,k) = R*w_hat*q_fd + qdot_leader(:,k);
                qddot_follower(:,k) = R*(w_hat^2 + alpha_hat)*q_fd + qddot_leader(:,k);
                qdddot_follower(:,k) = R*(w_hat^3 + zeta_hat + 3*w_hat*alpha_hat)*q_fd + qdddot_leader(:,k);
            end
            
            traj = Trajectory2D();
            traj.x = q_follower(1,:);
            traj.y = q_follower(2,:);
            traj.xdot = qdot_follower(1,:);
            traj.ydot = qdot_follower(2,:);
            traj.xddot = qddot_follower(1,:);
            traj.yddot = qddot_follower(2,:);
            traj.xdddot = qdddot_follower(1,:);
            traj.ydddot = qdddot_follower(2,:);
            traj.xddddot = zeros(1,length(traj.x));
            traj.yddddot = zeros(1,length(traj.y));
            traj.dt = traj_leader.dt;
            traj.update_new();
%             traj.w = (traj.xdot.*traj.yddot - traj.ydot.*traj.xddot)./sqrt(traj.x.^2 + traj.y.^2);
            t = 0:traj.dt:24.845;
            

        end
            

    end
end

