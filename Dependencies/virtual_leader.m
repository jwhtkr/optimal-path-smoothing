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
        planner;
    end
    
    methods
        function obj = virtual_leader(vehicle, n_agents)
            obj.vehicle = vehicle;
            obj.n_agents = n_agents;
            obj.planner = Unicycle2(obj.vehicle.x);
        end
        
        function initializePlots(obj, ax)
            obj.vehicle.sensor.initializePlots(ax);
            obj.vehicle.kinematics.c = 'r';
            obj.vehicle.kinematics.initializeStatePlot(ax, obj.vehicle.x);
            
            for k = 1:obj.n_agents
                q_fd = obj.getDesiredFollowerPosition(k);
                obj.agent_position(k) = plot(ax, q_fd(1), q_fd(2), 'ko', 'linewidth', 3);
                
                P = obj.getVoronoiBarrierLines(k);
                obj.voronoiBarrier(k) = plot(ax, P(1,:), P(2,:));
            end
            
        end
        
        function P = getVoronoiBarrierLines(obj,agent_num)
            q_l = obj.vehicle.x(obj.vehicle.q_ind);
            theta = obj.vehicle.x(obj.vehicle.th_ind);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            if obj.n_agents == 2
                    p1 = q_l + R*[2;0];
                    p2 = q_l - R*[2;0];
                    P = [p1,p2];
            else
                    phi = ((agent_num-1)*2*pi)/(obj.n_agents)+pi/obj.n_agents;
                    offset = [cos(phi) -sin(phi); sin(phi) cos(phi)];
                    
                    p1 = q_l;
                    p2 = q_l + offset*R*[2;0];
                    P = [p1,p2];
            end
                
        end
        
        function d = getVoronoiDistance(obj, x, agent_num)
            q_l = obj.vehicle.x(obj.vehicle.q_ind);
            theta = obj.vehicle.x(obj.vehicle.th_ind);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            
            q_f = x(obj.vehicle.q_ind)-q_l;
            
            if obj.n_agents == 2
                    n1 = R*[0;1];
                    
                    d = abs(dot(q_f,n1));
                    
                    
                    
            else
                    phi = ((agent_num-1)*2*pi)/(obj.n_agents)+pi/obj.n_agents;
                    offset = [cos(phi) -sin(phi); sin(phi) cos(phi)];
                    
                    p1 = q_l;
                    p2 = q_l + offset*R*[1;0];
                    P = [p1,p2];
            end
                
        end
        
        
        function q_fd = getDesiredFollowerPosition(obj, agent_num)
            q_l = obj.vehicle.x(obj.vehicle.q_ind);
            theta = obj.vehicle.x(obj.vehicle.th_ind);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            if obj.n_agents >=3
                phi = ((agent_num-1)*2*pi)/(obj.n_agents);
                offset = [cos(phi) -sin(phi); sin(phi) cos(phi)];
            end
            if obj.n_agents == 2
                phi = (agent_num-1)*pi+pi/2;
                offset = [cos(phi) -sin(phi); sin(phi) cos(phi)];
            end
                
            q_fd = q_l + offset*R*[1;0];
            
            
            if obj.n_agents == 1
                q_fd = q_l;
            end
        end
        
        function q_fd = getDesiredFollowerGoal(obj, agent_num)
            q_l = obj.leader_xT(obj.vehicle.q_ind);
            theta = obj.leader_xT(obj.vehicle.th_ind);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            if obj.n_agents >=3
                phi = ((agent_num-1)*2*pi)/(obj.n_agents);
                offset = [cos(phi) -sin(phi); sin(phi) cos(phi)];
            end
            if obj.n_agents == 2
                phi = (agent_num-1)*pi+pi/2;
                offset = [cos(phi) -sin(phi); sin(phi) cos(phi)];
            end
                
            q_fd = q_l + offset*R*[1;0];
            
            
            if obj.n_agents == 1
                q_fd = q_l;
            end
        end
        
        function plotFormation(obj)
            obj.vehicle.plotVehicle();
            
            for k = 1:obj.n_agents
                q_fd = obj.getDesiredFollowerPosition(k);
                set(obj.agent_position(k), 'xdata', q_fd(1), 'ydata', q_fd(2));
                
                P = obj.getVoronoiBarrierLines(k);
                set(obj.voronoiBarrier(k), 'xdata', P(1,:), 'ydata', P(2,:));
            end
        end
        
        function setLeaderTerminalState(obj,x,u)
%             z_sol = obj.integrate(@(t,z)obj.UnicycleDynamics(t,z,u), obj.vehicle.x, true);
            [obj.leader_traj, obj.leader_xT] = obj.planner.getTerminalPosition(x,u);
            obj.t_span = obj.planner.t_span;
%             obj.leader_xT = obj.leader_traj;
%             x = z_sol;
            
        end
        
        function x_i = getFutureLeaderState(obj,t)
            
        end
    end
    
end

