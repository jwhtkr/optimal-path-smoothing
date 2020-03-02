classdef MultiWallFollowScenario < MultiAgentScenario
    %MultiWallFollowScenario Implements several agents which follow a wall
    
    properties
        qd_mat % Stores the desired trajectory over time, used for plotting results,
               % The matrix has 2 x n_agents number of rows (2 for each
               % agent)
        agent_colors % n_agents x 3 matrix where each row represents the color of a different agent        
    end
    
    methods
        function obj = MultiWallFollowScenario(veh, world, x0)
            %MultiReferenceAvoidScenario Construct an instance of this class
            %   veh : instance to a vehicle class to be used to create the
            %   agents
            %   world: instance of the polygon world class
            %   waypoints: Cell structure of waypoints, one for each agent
            %   to be run
            %   x0: Cell structure of initial states, one for each agent to
            %   be run
            
            % Create each agent and its corresponding plotter
            n_agents = 1;
            agents = cell(n_agents, 1);
            plotters = {};
            agent_colors = distinguishable_colors(n_agents);
            
            % Create the agents
            veh_i = veh(x0{1});
            agents{1} = WallFollowAgent(veh_i, world, false);                
            
            % Create a vehicle plotter
            plotters{end+1} = SingleAgentPlotter(@(t)veh_i.getConfiguration(t), agent_colors(1,:));
            plotters{end+1} = VectorFieldPlotter(agents{1}.wall.line_vf);  
            plotters{end+1} = TwoDRangePlotter(veh_i);
            
            % Initialize the object
            obj = obj@MultiAgentScenario(agents, world, plotters, true);            
            obj.agent_colors = agent_colors;
        end
        
        function plotResults(obj)     
            % Plot the executed trajectory for each agent
            hold on;
            for i = 1:obj.n_agents
                % Get state indices
                x_ind = obj.state_ind{i}(1);
                y_ind = obj.state_ind{i}(2);
                
                % Plot the actual trajectory
                plot(obj.xmat(x_ind, :), obj.xmat(y_ind, :), 'linewidth', 2, 'color', obj.agent_colors(i,:));
            end
        end
    end
end



