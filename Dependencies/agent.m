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
        trajectory;
        leader;
        Q;
        dt
    end
    
    methods
        function obj = agent(vehicle, n_agents, agent_num, traj, dt)
            obj.vehicle = vehicle;
            obj.n_agents = n_agents;
            obj.Q = obj.DesiredFollowerPositions();
            obj.dt = dt;
            obj.planner = Unicycle2(obj.vehicle.x);
            obj.planner.agent_num = agent_num;
            if ~isempty(obj.Q)
                obj.planner.voronoi = voronoiWrapper(obj.Q);
            end
            if ~(agent_num == 0)
                obj.trajectory = traj(agent_num);
                obj.setInitialControlFromTraj(0);
                obj.planner.trajectory = obj.trajectory;
            end
        end
        
        function Q = DesiredFollowerPositions(obj)
            switch obj.n_agents
                case 0
                    Q = [];
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
        
        function u = trackControl(obj, t)
            traj = @(t) obj.trajectory.reference_traj(t);
            u = obj.vehicle.TrackTrajectoryApproximateDiffeomorphism(t, obj.vehicle.x, traj);
        end
        
        function u = MPC_output(obj, t, world)
            
            obj.planner.qd = obj.trajectory.reference_traj(t+obj.planner.tf); %obj.leader.getDesiredFollowerGoal(k);
%             obj.planner.leader_traj = obj.leader.trajectory;
            [xo,yo,do] = obj.vehicle.getObstacleDetections(world);
            obj.planner.setObstacles(xo,yo,do);
            % method to calculate desired velocities

            u_agent = obj.planner.minimize(t,obj.vehicle.x,obj.vehicle.u_init);
            obj.vehicle.v_d = u_agent(obj.planner.ind_a1);
            obj.vehicle.w_d = u_agent(obj.planner.ind_alpha1);
            obj.vehicle.u_init = u_agent;
            u = obj.vehicle.velocityControl(obj.vehicle.v_d, obj.vehicle.w_d, obj.vehicle.x);
        end
        
        function setInitialControlFromTraj(obj,t)
            ind1 = round(t/obj.dt+1,0);
            ind2 = round((obj.planner.T/3+t)/obj.dt+1,0);
            ind3 = round((obj.planner.T*2/3+t)/obj.dt+1,0);
            obj.vehicle.u_init = [obj.trajectory.v(ind1); obj.trajectory.w(ind1); obj.planner.T/3; ...
                    obj.trajectory.v(ind2); obj.trajectory.w(ind2); obj.planner.T*2/3; ...
                    obj.trajectory.v(ind3); obj.trajectory.w(ind3)];
        end
        
    end
    
end