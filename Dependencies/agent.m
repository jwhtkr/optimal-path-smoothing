classdef agent < handle
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
        agent_num;
        agent_position;
        planner;
        vl;
        Q
    end
    
    methods
        function obj = agent(vehicle, n_agents, agent_num)
            obj.vehicle = vehicle;
            obj.planner = Unicycle2(obj.vehicle.x);
            obj.planner.agent_num = agent_num;
            obj.n_agents = n_agents;
            obj.Q = obj.DesiredFollowerPositions();
            obj.planner.voronoi = voronoiWrapper(obj.Q);
        end
        
        function Q = DesiredFollowerPositions(obj)
            switch obj.n_agents
                case 2
                    Q = [-1 1; 0 0];
                case 3
                    Q = [1 cos(2*pi/3) cos(4*pi/3); 0 sin(2*pi/3) sin(4*pi/3)];
                case 4
                    Q = [1.5 0 0 -1.5; 0 1 -1 0];
                otherwise
                    disp('Unsupported Number of Agents, define formation in DesiredFollowerPositions function')
            end
        end
    end
    
end