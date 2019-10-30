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
    end
    
    methods
        function obj = virtual_leader(vehicle, n_agents)
            obj.vehicle = vehicle;
            obj.n_agents = n_agents;
            obj.agent_num = 0;
            obj.agent = agent(vehicle, n_agents, obj.agent_num);
        end       
        
        function q_fd = getDesiredFollowerPosition(obj, agent_num)
            obj.updateFormation();
            q_fd = obj.Q(:,agent_num);
        end

        function q_fd = getDesiredFollowerGoal(obj, agent_num)
            q_l = obj.leader_xT(obj.vehicle.q_ind);
            theta = obj.leader_xT(obj.vehicle.th_ind);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            
            q_fd = q_l + R*obj.agent.Q(:,agent_num);
        end
        
        function setLeaderTerminalState(obj,x,u)
            [obj.leader_traj, obj.leader_xT] = obj.agent.planner.getTerminalPosition(x,u);
            obj.t_span = obj.agent.planner.t_span;
        end
        
        function initializePlots(obj, ax)
            obj.vehicle.sensor.initializePlots(ax);
            obj.vehicle.kinematics.c = 'r';
            obj.vehicle.kinematics.initializeStatePlot(ax, obj.vehicle.x);
            obj.updateFormation();
            
            obj.agent_position = plot(ax, obj.Q(1,:), obj.Q(2,:), 'ko', 'linewidth', 3);
            
            for k = 1:length(obj.V)/2
                obj.voronoiBarrier(k) = plot(ax, obj.V(1,2*k-1:2*k), obj.V(2,2*k-1:2*k),'k');
            end
        end
        
        function plotFormation(obj)
            obj.vehicle.plotVehicle();
            obj.updateFormation();
            
            set(obj.agent_position, 'xdata', obj.Q(1,:), 'ydata', obj.Q(2,:));
            for k = 1:length(obj.V)/2               
                set(obj.voronoiBarrier(k), 'xdata', obj.V(1,2*k-1:2*k), 'ydata', obj.V(2,2*k-1:2*k));
            end
            
            
        end
        
        
        function updateFormation(obj)
            q_l = obj.vehicle.x(obj.vehicle.q_ind);
            theta = obj.vehicle.x(obj.vehicle.th_ind);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            
            obj.Q = R*obj.agent.Q + q_l*ones(1,obj.n_agents);
            obj.V = R*obj.agent.planner.voronoi.V + q_l*ones(1,length(obj.agent.planner.voronoi.V));
        end
    end
end

