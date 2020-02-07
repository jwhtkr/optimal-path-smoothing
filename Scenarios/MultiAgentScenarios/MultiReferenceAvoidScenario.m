classdef MultiReferenceAvoidScenario < MultiAgentScenario
    %MultiReferenceAvoidScenario Implements several agents which track a
    %reference as individual agents
    
    properties
        qd_mat % Stores the desired trajectory over time, used for plotting results,
               % The matrix has 2 x n_agents number of rows (2 for each
               % agent)
        traj_follow = {} % A cell structure containing the desired trajectory for each follower
        traj_eps = {} % A cell structure containing the desired trajectory for the epsilon point of each follower
        agent_colors % n_agents x 3 matrix where each row represents the color of a different agent
    end
    
    methods
        function obj = MultiReferenceAvoidScenario(veh, world, waypoints, x0, Q)
            %MultiReferenceAvoidScenario Construct an instance of this class
            %   veh : instance to a vehicle class to be used to create the
            %   agents
            %   world: instance of the polygon world class
            %   waypoints: an array of path points in the desired order (e.g., [q0,q1,...,qn] 
            %              where qi = [x; y]
            %   x0: Cell structure of initial states, one for each agent to
            %   be run (size n_agents)
            %   Q: Offsets from virtual leader of all the agents
            %      2 x n_agents where each column is the desired offset
            
            
            % Define desired trajectory pamameters
            dt = 0.01;
            vd = 1;
            k_max = 0.35; % Maximum curvature
            sig_max = 0.35; % Maximum change in curvature
            
            % Create the virtual leader trajectory
            vl_traj = TrajUtil.createClothoidTrajectory(waypoints, vd, dt, k_max, sig_max);
            
            % Create a unique color for each agent
            n_agents = length(x0);
            agent_colors = distinguishable_colors(n_agents);
            
            % Create each agent and its corresponding plotter
            agents = cell(n_agents, 1);
            plotters = {};
            for i = 1:length(agents)
                % Create the agent
                veh_i = veh(x0{i}); veh_i.use_dim_eps = false;
                traj_follow{i} = TrajUtil.createOffsetTrajectory(vl_traj, Q(:,i)); % Desired trajectory for follower
                traj_eps{i} = TrajUtil.createOffsetTrajectory(traj_follow{i}, [veh_i.eps_path; 0]); % Desired trajectory for the epsilon point of the follower
                agents{i} = ReferenceAvoidAgent(veh_i, world, traj_follow{i}, traj_eps{i});
                
                % Create a vehicle plotter
                plotters{end+1} = SingleAgentPlotter(veh_i, agent_colors(i,:));
                
                % Createa a plotter for the desired position
                %plotters{end+1} = PositionPlotter(@(t)agents{i}.ReferenceTraj(t));
                plotters{end+1} = PositionPlotter(@(t)traj_follow{i}.reference_traj(t), agent_colors(i,:));
                plotters{end+1} = PositionPlotter(@(t)traj_eps{i}.reference_traj(t), agent_colors(i,:));
            end
            
            % Initialize the object
            obj = obj@MultiAgentScenario(agents, world, plotters, true);
            
            % Store object variables
            obj.traj_follow = traj_follow;
            obj.traj_eps = traj_eps;
            obj.agent_colors = agent_colors;
        end
        
        function initializePlots(obj)
            % Plot normal plots
            initializePlots@MultiAgentScenario(obj);
            
            % Get indices for plotting the epsilon trajectory
            len = length(obj.tmat);
            ind = zeros(1,len);
            for k = 1:len
                ind(k) = obj.traj_eps{1}.getIndex(obj.tmat(k));
            end
            
            % Plot the reference trajectory for each agent
            hold on;
            obj.qd_mat = zeros(2*obj.n_agents, len);
            ind_q_i = 1:2; % Iteratively stores the position indices for each agent
            for i = 1:obj.n_agents
                % Plot the epsilon trajectory
                plot(obj.traj_eps{i}.x(ind), obj.traj_eps{i}.y(ind), ':', 'linewidth', 1, 'color', obj.agent_colors(i,:));
                
                % Plot the actual trajectory
                plot(obj.traj_follow{i}.x(ind), obj.traj_follow{i}.y(ind), ':', 'linewidth', 2, 'color', obj.agent_colors(i,:));
                
                % Store the desired states
                obj.qd_mat(ind_q_i, :) = [obj.traj_follow{i}.x(ind); ...
                                          obj.traj_follow{i}.y(ind)];
                                      
                % Update the position indices
                ind_q_i = ind_q_i + 2; % Add two to adjust for the two positions
            end
        end
        
        function plotResults(obj)
            ind_q_d = 1:2; % Stores the indices of agent i within q_d
            for i = 1:obj.n_agents
                % Create a figure for the actual and desired positions
                figure;
                
                % Get state indices
                x_ind = obj.state_ind{i}(1);
                y_ind = obj.state_ind{i}(2);
                
                % Plot the x position vs desired position
                subplot(2, 1, 1);
                plot(obj.tmat, obj.qd_mat(ind_q_d(1), :), ':r', 'linewidth', 3); hold on;
                plot(obj.tmat, obj.xmat(x_ind, :), 'b', 'linewidth', 2);
                ylabel('x position');
                
                % Plot the y position vs desired position
                subplot(2, 1, 2);
                plot(obj.tmat, obj.qd_mat(ind_q_d(2), :), ':r', 'linewidth', 3); hold on;
                plot(obj.tmat, obj.xmat(y_ind, :), 'b', 'linewidth', 2);
                ylabel('y position');
                xlabel('time (s)');
                
                % Upate the position indices for the next agent
                ind_q_d = ind_q_d + 2;
            end
        end
    end
end



